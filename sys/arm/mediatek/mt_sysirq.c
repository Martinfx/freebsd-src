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

/*
 * MediaTek system IRQ controller.
 *
 * The block sits between the peripherals and the GIC and only deals with
 * polarity: one bit per SPI in the INTPOL registers inverts the line, so a
 * peripheral described as active low or falling edge in the device tree
 * reaches the GIC as active high or rising edge.  The interrupts themselves
 * belong to the GIC.  This driver maps a request onto the GIC interrupt
 * source, programs the polarity bit, takes ownership of the source and then
 * forwards every operation to the GIC with a translated specification.
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
#include <machine/intr.h>

#include <dev/fdt/fdt_intr.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dt-bindings/interrupt-controller/irq.h>
#include "pic_if.h"

#define    IRQ_TYPE_SENSE_MASK    0x0000000f

static struct ofw_compat_data compat_data[] = {
	{ "mediatek,mt6577-sysirq",	1 },
	{ NULL,				0 }
};

struct mt_sysirq_sc {
    device_t dev;
    device_t parent;
    int rid;
    int nirq;
    struct resource *res;
    struct intr_map_data_fdt *parent_map_data;
};

static struct intr_map_data *
mt_sysirq_convert_map_data(struct mt_sysirq_sc *sc,
                               struct intr_map_data *data) {
        struct intr_map_data_fdt *daf;
        uint32_t type;
        int irq;

        if (data == NULL || data->type != INTR_MAP_DATA_FDT)
                return (NULL);

        daf = (struct intr_map_data_fdt *) data;

        if (daf->ncells != 3) {
                device_printf(sc->dev, "%s: bad ncells=%d\n",
                    __func__, daf->ncells);
                return (NULL);
        }

        irq = daf->cells[1];
        if (irq < 0 || irq >= sc->nirq) {
                device_printf(sc->dev, "%s: irq %d out of range [0..%d)\n",
                    __func__, irq, sc->nirq);
                return (NULL);
        }

        type = daf->cells[2] & IRQ_TYPE_SENSE_MASK;
        if (type == IRQ_TYPE_LEVEL_LOW)
                type = IRQ_TYPE_LEVEL_HIGH;
        else if (type == IRQ_TYPE_EDGE_FALLING)
                type = IRQ_TYPE_EDGE_RISING;

        /* Build a GIC compatible mapping (3 cells: SPI, num, type). */
        sc->parent_map_data->ncells = 3;
        sc->parent_map_data->cells[0] = 0;        /* GIC_SPI */
        sc->parent_map_data->cells[1] = daf->cells[1];
        sc->parent_map_data->cells[2] = type;

        return ((struct intr_map_data *) sc->parent_map_data);
}

static int
mt_sysirq_activate_intr(device_t dev, struct intr_irqsrc *isrc,
                            struct resource *res, struct intr_map_data *data) {
        struct mt_sysirq_sc *sc = device_get_softc(dev);

        data = mt_sysirq_convert_map_data(sc, data);
        if (data == NULL)
                return (EINVAL);

        return (PIC_ACTIVATE_INTR(sc->parent, isrc, res, data));
}

static void
mt_sysirq_enable_intr(device_t dev, struct intr_irqsrc *isrc) {
        struct mt_sysirq_sc *sc = device_get_softc(dev);

        PIC_ENABLE_INTR(sc->parent, isrc);
}

static void
mt_sysirq_disable_intr(device_t dev, struct intr_irqsrc *isrc) {
        struct mt_sysirq_sc *sc = device_get_softc(dev);

        PIC_DISABLE_INTR(sc->parent, isrc);
}

static int
mt_sysirq_map_intr(device_t dev, struct intr_map_data *data,
                       struct intr_irqsrc **isrcp) {
        struct mt_sysirq_sc *sc = device_get_softc(dev);
        int ret;

        if (data->type != INTR_MAP_DATA_FDT)
                return (ENOTSUP);

        data = mt_sysirq_convert_map_data(sc, data);
        if (data == NULL)
                return (EINVAL);

        ret = PIC_MAP_INTR(sc->parent, data, isrcp);
        if (ret == 0 && *isrcp != NULL)
                (*isrcp)->isrc_dev = sc->dev;

        return (ret);
}

static int
mt_sysirq_deactivate_intr(device_t dev, struct intr_irqsrc *isrc,
                              struct resource *res, struct intr_map_data *data) {
        struct mt_sysirq_sc *sc = device_get_softc(dev);

        data = mt_sysirq_convert_map_data(sc, data);
        if (data == NULL)
                return (EINVAL);

        return (PIC_DEACTIVATE_INTR(sc->parent, isrc, res, data));
}

static int
mt_sysirq_setup_intr(device_t dev, struct intr_irqsrc *isrc,
                         struct resource *res, struct intr_map_data *data) {
        struct mt_sysirq_sc *sc = device_get_softc(dev);
        struct intr_map_data_fdt *daf;
        uint32_t reg, val, type;
        int irq, bit;

        if (data == NULL || data->type != INTR_MAP_DATA_FDT)
                return (EINVAL);

        /* Read the original (un-inverted) request before converting it. */
        daf = (struct intr_map_data_fdt *) data;
        if (daf->ncells != 3)
                return (EINVAL);
        irq = daf->cells[1];
        type = daf->cells[2] & IRQ_TYPE_SENSE_MASK;
        if (irq < 0 || irq >= sc->nirq)
                return (EINVAL);

        reg = (irq / 32) * 4;
        bit = irq % 32;

        val = bus_read_4(sc->res, reg);
        if (type == IRQ_TYPE_LEVEL_LOW || type == IRQ_TYPE_EDGE_FALLING)
                val |= (1U << bit);    /* Invert polarity for the GIC. */
        else
                val &= ~(1U << bit);
        bus_write_4(sc->res, reg, val);

        data = mt_sysirq_convert_map_data(sc, data);
        if (data == NULL)
                return (EINVAL);

        return (PIC_SETUP_INTR(sc->parent, isrc, res, data));
}

static int
mt_sysirq_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
                            struct resource *res, struct intr_map_data *data) {
        struct mt_sysirq_sc *sc = device_get_softc(dev);
        struct intr_map_data_fdt *daf;
        uint32_t reg, val;
        int irq, bit;

        if (data == NULL || data->type != INTR_MAP_DATA_FDT)
                return (EINVAL);

        daf = (struct intr_map_data_fdt *) data;
        if (daf->ncells != 3)
                return (EINVAL);
        irq = daf->cells[1];
        if (irq < 0 || irq >= sc->nirq)
                return (EINVAL);

        reg = (irq / 32) * 4;
        bit = irq % 32;

        val = bus_read_4(sc->res, reg);
        val &= ~(1U << bit);        /* Clear the polarity invert bit. */
        bus_write_4(sc->res, reg, val);

        data = mt_sysirq_convert_map_data(sc, data);
        if (data == NULL)
                return (EINVAL);

        return (PIC_TEARDOWN_INTR(sc->parent, isrc, res, data));
}

static void
mt_sysirq_pre_ithread(device_t dev, struct intr_irqsrc *isrc) {
        struct mt_sysirq_sc *sc = device_get_softc(dev);

        PIC_PRE_ITHREAD(sc->parent, isrc);
}

static void
mt_sysirq_post_ithread(device_t dev, struct intr_irqsrc *isrc) {
        struct mt_sysirq_sc *sc = device_get_softc(dev);

        PIC_POST_ITHREAD(sc->parent, isrc);
}

static void
mt_sysirq_post_filter(device_t dev, struct intr_irqsrc *isrc) {
        struct mt_sysirq_sc *sc = device_get_softc(dev);

        PIC_POST_FILTER(sc->parent, isrc);
}

static int
mt_sysirq_probe(device_t dev) {

        if (!ofw_bus_status_okay(dev))
                return (ENXIO);

        if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
                return (ENXIO);

        device_set_desc(dev, "MediaTek SYSIRQ Controller");
        return (BUS_PROBE_DEFAULT);
}

static int
mt_sysirq_attach(device_t dev) {
        struct mt_sysirq_sc *sc;
        phandle_t node, xref, intr_parent;

        sc = device_get_softc(dev);
        sc->dev = dev;
        sc->rid = 0;
        node = ofw_bus_get_node(dev);

        sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->rid,
            RF_ACTIVE);
        if (sc->res == NULL) {
                device_printf(dev, "Cannot allocate memory resource\n");
                return (ENXIO);
        }

        /* Each 32-bit register controls 32 interrupts. */
        sc->nirq = rman_get_size(sc->res) * 8;

        if ((intr_parent = ofw_bus_find_iparent(node)) == 0) {
                device_printf(dev, "Cannot find interrupt-parent\n");
                goto fail;
        }

        sc->parent = OF_device_from_xref(intr_parent);
        if (sc->parent == NULL) {
                device_printf(dev, "Cannot find parent controller (GIC)\n");
                goto fail;
        }

        xref = OF_xref_from_node(node);
        if (intr_pic_register(dev, xref) == NULL) {
                device_printf(dev, "Cannot register PIC\n");
                goto fail;
        }

        /* GIC compatible mapping entry (3 cells). */
        sc->parent_map_data = (struct intr_map_data_fdt *) intr_alloc_map_data(
            INTR_MAP_DATA_FDT,
            sizeof(struct intr_map_data_fdt) + 3 * sizeof(phandle_t),
            M_WAITOK | M_ZERO);

        OF_device_register_xref(xref, dev);

        return (0);

fail:
        bus_release_resource(dev, SYS_RES_MEMORY, sc->rid, sc->res);
        return (ENXIO);
}

static device_method_t mt_sysirq_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mt_sysirq_probe),
	DEVMETHOD(device_attach,	mt_sysirq_attach),
	//DEVMETHOD(device_detach,	mt_sysirq_detach),

	/* Interrupt controller interface */
	DEVMETHOD(pic_map_intr,		mt_sysirq_map_intr),
	DEVMETHOD(pic_activate_intr,	mt_sysirq_activate_intr),
	DEVMETHOD(pic_deactivate_intr,	mt_sysirq_deactivate_intr),
	DEVMETHOD(pic_setup_intr,	mt_sysirq_setup_intr),
	DEVMETHOD(pic_teardown_intr,	mt_sysirq_teardown_intr),
	DEVMETHOD(pic_enable_intr,	mt_sysirq_enable_intr),
	DEVMETHOD(pic_disable_intr,	mt_sysirq_disable_intr),
	DEVMETHOD(pic_pre_ithread,	mt_sysirq_pre_ithread),
	DEVMETHOD(pic_post_ithread,	mt_sysirq_post_ithread),
	DEVMETHOD(pic_post_filter,	mt_sysirq_post_filter),

	DEVMETHOD_END
};

static DEFINE_CLASS_0(mt_sysirq, mt_sysirq_driver, mt_sysirq_methods,
    sizeof(struct mt_sysirq_sc));

/* Right after the GIC, before anything that asks for an interrupt. */
EARLY_DRIVER_MODULE(mt_sysirq, simplebus, mt_sysirq_driver, NULL, NULL,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE + 1);
EARLY_DRIVER_MODULE(mt_sysirq, ofwbus, mt_sysirq_driver, NULL, NULL,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE + 1);
