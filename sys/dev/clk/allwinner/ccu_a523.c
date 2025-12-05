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
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>

#include <dev/fdt/simplebus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk_div.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_mux.h>

#include <dev/clk/allwinner/aw_ccung.h>

#include <dt-bindings/clock/sun55i-a523-ccu.h>
#include <dt-bindings/reset/sun55i-a523-ccu.h>

static const char *pll_ddr_parent[] = { "hosc" };
NKMP_CLK(pll_ddr_clk,
        CLK_PLL_DDR0,				/* id */
        "pll-ddr0", pll_ddr_parent, nitems(pll_ddr_parent)		/* name, parents */
        0x010,					/* offset */
        8, 8, 11, 0,					/* n factor */
        0, 0, 0, 0,		            /* k factor (fake) */
        1, 1, 0, 0,					/* m factor */
        0, 1, 0, 0,	/* p factor */
        28, 0,					/* lock */
        0,
        AW_CLK_HAS_LOCK );		/* flags */

static struct aw_ccung_clk a523_ccu_clks[] = {
        {.type = AW_CLK_NKMP, .clk.nkmp = &pll_ddr_clk},
};

static int
ccu_a523_probe(device_t dev)
{

    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (!ofw_bus_is_compatible(dev, "allwinner,sun55i-a523-ccu"))
        return (ENXIO);

    device_set_desc(dev, "Allwinner A523 Clock Control Unit");
    return (BUS_PROBE_DEFAULT);
}

static int
ccu_a523_attach(device_t dev)
{
    struct aw_ccung_softc *sc;

    sc = device_get_softc(dev);

    //sc->resets = a64_ccu_resets;
    // sc->nresets = nitems(a64_ccu_resets);
    //sc->gates = a64_ccu_gates;
    //sc->ngates = nitems(a64_ccu_gates);
    sc->clks = a523_ccu_clks;
    sc->nclks = nitems(a523_ccu_clks);
    //sc->clk_init = a64_init_clks;
    //sc->n_clk_init = nitems(a64_init_clks);

    return (aw_ccung_attach(dev));
}

static device_method_t ccu_a523_methods[] = {
        /* Device interface */
        DEVMETHOD(device_probe,		ccu_a523_probe),
        DEVMETHOD(device_attach,	ccu_a523_attach),

        DEVMETHOD_END
};

DEFINE_CLASS_1(ccu_a523, ccu_a523_driver, ccu_a523_methods,
sizeof(struct aw_ccung_softc), aw_ccung_driver);

EARLY_DRIVER_MODULE(ccu_a523, simplebus, ccu_a523_driver, 0, 0,
BUS_PASS_RESOURCE + BUS_PASS_ORDER_MIDDLE);