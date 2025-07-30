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
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <machine/bus.h>

#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include "mt7622_clk_pll.h"

struct mt7622_pll_sc {
    device_t clkdev;
    uint32_t base_reg;
    uint32_t pwr_reg;
    uint32_t en_mask;
    uint32_t pd_reg;
    uint32_t tuner_reg;
    uint32_t tuner_en_reg;
    uint8_t tuner_en_bit;
    int pd_shift;
    unsigned int flags;
    uint64_t fmin;
    uint64_t fmax;
    int pcwbits;
    int pcwibits;
    uint32_t pcw_reg;
    int pcw_shift;
    uint32_t pcw_chg_reg;
    //struct clk_pll_div_table *div_table;
};

static int
mt7622_clk_pll_init(struct clknode *clk, device_t dev)
{
    clknode_init_parent_idx(clk, 0);

    return (0);
}

static int
mt7622_clk_pll_recalc_freq(struct clknode *clk, uint64_t *freq)
{
    return(0);
}

static int
mt7622_clk_pll_set_freq(struct clknode *clknode, uint64_t fin, uint64_t *fout,
                        int flags, int *stop)
{
    struct mt7622_pll_sc *sc = NULL;
    sc = device_get_softc(clknode_get_device(clknode));
    device_printf(sc->clkdev, "Set pll frequency %s - %d \n", __func__, __LINE__ );
    if(sc == NULL) {
        return (ENXIO);
    }

    device_printf(sc->clkdev, "%s: %s requested freq: %lu, input freq: %lu\n", __func__,
            clknode_get_name(clknode), *fout, fin);


    return(0);
}


static clknode_method_t mt7622_pllnode_methods[] = {
        /* Device interface */
        CLKNODEMETHOD(clknode_init,		mt7622_clk_pll_init),
        CLKNODEMETHOD(clknode_recalc_freq,	mt7622_clk_pll_recalc_freq),
        CLKNODEMETHOD(clknode_set_freq,		mt7622_clk_pll_set_freq),

        CLKNODEMETHOD_END
};

DEFINE_CLASS_1(mt7622_pllnode, mt7622_pllnode_class, mt7622_pllnode_methods,
sizeof(struct mt7622_pll_sc), clknode_class);

int
mt7622_clk_pll_register(struct clkdom *clkdom, struct clk_pll_def *clkdef)
{
    struct clknode *clk = NULL;
    struct mt7622_pll_sc *sc;

    clk = clknode_create(clkdom, &mt7622_pllnode_class, &clkdef->clkdef);
    if (clk == NULL) {
        return (ENXIO);
    }

    sc = clknode_get_softc(clk);
    sc->clkdev = clknode_get_device(clk);
    sc->base_reg = clkdef->base_reg;
    sc->pwr_reg = clkdef->pwr_reg;
    sc->en_mask = clkdef->en_mask;
    sc->pd_reg = clkdef->pd_reg;
    sc->tuner_reg = clkdef->tuner_reg;
    sc->tuner_en_reg = clkdef->tuner_en_reg;
    sc->tuner_en_bit = clkdef->tuner_en_bit;
    sc->pd_shift = clkdef->pd_shift;
    sc->flags = clkdef->flags;
    sc->fmin = clkdef->fmin;
    sc->fmax = clkdef->fmax;
    sc->pcwbits = clkdef->pcwbits;
    sc->pcwibits = clkdef->pcwibits;
    sc->pcw_reg = clkdef->pcw_reg;
    sc->pcw_shift = clkdef->pcw_shift;
    sc->pcw_chg_reg = clkdef->pcw_chg_reg;
    clknode_register(clkdom, clk);
    device_printf(sc->clkdev, "PLL clocks registered. %s - %d: \n", __func__ , __LINE__ );
    return (0);
}