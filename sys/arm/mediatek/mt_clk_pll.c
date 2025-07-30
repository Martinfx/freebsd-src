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

#include <dev/clk/clk.h>
#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include "clkdev_if.h"
#include "mdtk_clk.h"
#include "mt_clk_pll.h"

#define RD4(clk, reg, val)    CLKDEV_READ_4(clknode_get_device(clk), reg, val)
#define WR4(clk, reg, val)    CLKDEV_WRITE_4(clknode_get_device(clk), reg, val)
#define    DEVICE_LOCK(clk)    CLKDEV_DEVICE_LOCK(clknode_get_device(clk))
#define    DEVICE_UNLOCK(clk)    CLKDEV_DEVICE_UNLOCK(clknode_get_device(clk))

/* Mask of bits [h:l]. */
#define MT_MASK(h, l)        (((~(uint32_t)0) >> (31 - ((h) - (l)))) << (l))
#define MT_POSTDIV_MASK        0x7u
#define MT_PLL_PWR_ON        (1u << 0)    /* in pll_pwr_reg */
#define MT_PLL_ISO_EN        (1u << 1)    /* in pll_pwr_reg */
#define MT_PLL_PCW_CHG_BIT    31
#define MT_AUDPLL_TUNER_EN    (1u << 31)
#define MT_PLL_REG_CON0        0x0u
/* Default number of integer bits when pll_pcwibits is not provided. */
#define MT_PLL_INTEGER_BITS    7

/*
 * pll_flags bit.  Keep this in sync with the value used in the board PLL
 * table; ideally move it to mt_clk_pll.h next to struct clk_pll_def.
 */
#define MT_PLL_FLAG_RST_BAR    (1u << 0)

struct mt_pll_clknode_softc {
    bus_size_t pll_base_reg;
    bus_size_t pll_pwr_reg;
    uint32_t pll_flags;
    uint64_t pll_fmin;
    uint64_t pll_fmax;
    uint32_t pll_en_mask;
    bus_size_t pll_pd_reg;
    bus_size_t pll_tuner_reg;
    bus_size_t pll_tuner_en_reg;
    uint32_t pll_tuner_en_bit;
    int pll_pd_shift;
    int pll_pcwbits;
    int pll_pcwibits;
    bus_size_t pll_pcw_reg;
    uint32_t pll_pcw_chg_reg;
    int pll_pcw_shift;
    struct div_table *pll_div_table;
    uint32_t pll_rst_bar_mask;
};

/*
* Compute the output frequency of the PLL from the raw PCW and postdiv
* values.  Mirrors Linux __mtk_pll_recalc_rate():
*   vco = fin * pcw / 2^pcwfbits   (rounded up if any fractional bit set)
*   out = ceil(vco / postdiv)
*/
static uint64_t
mt_pll_calc_freq(struct mt_pll_clknode_softc *sc, uint64_t fin,
                 uint32_t pcw, uint32_t postdiv) {
    int ibits, pcwfbits;
    uint64_t vco;
    uint8_t c = 0;

    ibits = sc->pll_pcwibits ? sc->pll_pcwibits : MT_PLL_INTEGER_BITS;
    pcwfbits = (sc->pll_pcwbits > ibits) ? sc->pll_pcwbits - ibits : 0;

    vco = fin * (uint64_t) pcw;
    if (pcwfbits != 0 && (vco & MT_MASK(pcwfbits - 1, 0)) != 0)
        c = 1;

    vco >>= pcwfbits;
    if (c != 0)
        vco++;

    return (vco + postdiv - 1) / postdiv;
}

static int
mt_clk_pll_init(struct clknode *clk, device_t dev) {
    struct mt_pll_clknode_softc *sc;
    uint32_t reg;

    sc = clknode_get_softc(clk);

    /* Power the PLL on. */
    RD4(clk, sc->pll_pwr_reg, &reg);
    reg |= MT_PLL_PWR_ON;
    WR4(clk, sc->pll_pwr_reg, reg);
    DELAY(1);

    /* Take it out of isolation. */
    RD4(clk, sc->pll_pwr_reg, &reg);
    reg &= ~MT_PLL_ISO_EN;
    WR4(clk, sc->pll_pwr_reg, reg);
    DELAY(1);

    /* Enable the PLL (EN bit(s) in CON0). */
    if (sc->pll_en_mask != 0) {
        RD4(clk, sc->pll_base_reg + MT_PLL_REG_CON0, &reg);
        reg |= sc->pll_en_mask;
        WR4(clk, sc->pll_base_reg + MT_PLL_REG_CON0, reg);
    }

    /* Enable the tuner if this PLL has one. */
    if (sc->pll_tuner_en_reg != 0) {
        RD4(clk, sc->pll_tuner_en_reg, &reg);
        reg |= (1u << sc->pll_tuner_en_bit);
        WR4(clk, sc->pll_tuner_en_reg, reg);
    } else if (sc->pll_tuner_reg != 0) {
        RD4(clk, sc->pll_tuner_reg, &reg);
        reg |= MT_AUDPLL_TUNER_EN;
        WR4(clk, sc->pll_tuner_reg, reg);
    }

    /* Wait for the PLL to lock before releasing the reset bar. */
    DELAY(20);

    if ((sc->pll_flags & MT_PLL_FLAG_RST_BAR) != 0) {
        RD4(clk, sc->pll_base_reg + MT_PLL_REG_CON0, &reg);
        reg |= sc->pll_rst_bar_mask;
        WR4(clk, sc->pll_base_reg + MT_PLL_REG_CON0, reg);
    }

    clknode_init_parent_idx(clk, 0);
    return (0);
}

static int
mt_clk_pll_recalc_freq(struct clknode *clk, uint64_t *freq) {
    struct mt_pll_clknode_softc *sc;
    uint32_t reg, postdiv_sel, postdiv, pcw;

    sc = clknode_get_softc(clk);

    DEVICE_LOCK(clk);

    RD4(clk, sc->pll_pd_reg, &reg);
    postdiv_sel = (reg >> sc->pll_pd_shift) & MT_POSTDIV_MASK;
    postdiv = 1u << postdiv_sel;

    RD4(clk, sc->pll_pcw_reg, &reg);
    pcw = (reg >> sc->pll_pcw_shift) & MT_MASK(sc->pll_pcwbits - 1, 0);

    DEVICE_UNLOCK(clk);

    /* On entry *freq is the parent (reference) frequency. */
    *freq = mt_pll_calc_freq(sc, *freq, pcw, postdiv);
    return (0);
}

static int
mt_clk_pll_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
                    int flags, int *stop) {
    struct mt_pll_clknode_softc *sc;
    struct div_table *table;
    uint64_t freq, pcw, result;
    uint32_t reg, postdiv, pcwmask = 0, val;
    int ibits;

    sc = clknode_get_softc(clk);
    table = sc->pll_div_table;

    freq = *fout;
    if (freq > sc->pll_fmax)
        freq = sc->pll_fmax;

    /* Pick the postdiv: val is log2(postdiv). */
    if (table != NULL) {
        if (freq > table[0].freq)
            freq = table[0].freq;
        for (val = 0; table[val + 1].freq != 0; val++) {
            if (freq > table[val + 1].freq)
                break;
        }
        postdiv = 1u << val;
    } else {
        for (val = 0; val < 5; val++) {
            postdiv = 1u << val;
            if (freq * postdiv >= sc->pll_fmin)
                break;
        }
    }

    /* pcw = freq * postdiv * 2^pcwfbits / fin */
    ibits = sc->pll_pcwibits ? sc->pll_pcwibits : MT_PLL_INTEGER_BITS;
    pcw = (freq << val) << (sc->pll_pcwbits - ibits);
    pcw /= fin;

    /* Report the frequency we would actually produce. */
    result = mt_pll_calc_freq(sc, fin, (uint32_t) pcw & pcwmask, postdiv);

    if ((flags & CLK_SET_DRYRUN) != 0) {
        *fout = result;
        *stop = 1;
        return (0);
    }

    if ((result < *fout) && (flags & CLK_SET_ROUND_DOWN) != 0) {
        *stop = 1;
        return (ERANGE);
    }
    if ((result > *fout) && (flags & CLK_SET_ROUND_UP) != 0) {
        *stop = 1;
        return (ERANGE);
    }

    pcwmask = MT_MASK(sc->pll_pcwbits - 1, 0);

    DEVICE_LOCK(clk);

    RD4(clk, sc->pll_pd_reg, &reg);
    reg &= ~(MT_POSTDIV_MASK << sc->pll_pd_shift);
    reg |= val << sc->pll_pd_shift;
    WR4(clk, sc->pll_pd_reg, reg);

    RD4(clk, sc->pll_pcw_reg, &reg);
    reg &= ~(pcwmask << sc->pll_pcw_shift);
    reg |= ((uint32_t) pcw & pcwmask) << sc->pll_pcw_shift;
    WR4(clk, sc->pll_pcw_reg, reg);

    RD4(clk, sc->pll_pcw_chg_reg, &reg);
    reg |= (1u << MT_PLL_PCW_CHG_BIT);
    WR4(clk, sc->pll_pcw_chg_reg, reg);

    DEVICE_UNLOCK(clk);

    *fout = result;
    *stop = 1;
    return (0);
}

static clknode_method_t mt_pllnode_methods[] = {
        CLKNODEMETHOD(clknode_init, mt_clk_pll_init),
        CLKNODEMETHOD(clknode_recalc_freq, mt_clk_pll_recalc_freq),
        CLKNODEMETHOD(clknode_set_freq, mt_clk_pll_set_freq),
        CLKNODEMETHOD_END
};

DEFINE_CLASS_1(mt_pllnode, mt_pllnode_class, mt_pllnode_methods,
sizeof(struct mt_pll_clknode_softc), clknode_class);

int
mt_clk_pll_register(struct clkdom *clkdom, struct clk_pll_def *clkdef) {
    struct clknode *clk;
    struct mt_pll_clknode_softc *sc;

    clk = clknode_create(clkdom, &mt_pllnode_class, &clkdef->clkdef);
    if (clk == NULL)
        return (ENXIO);

    sc = clknode_get_softc(clk);
    sc->pll_base_reg = clkdef->pll_base_reg;
    sc->pll_pwr_reg = clkdef->pll_pwr_reg;
    sc->pll_flags = clkdef->pll_flags;
    sc->pll_fmin = clkdef->pll_fmin;
    sc->pll_fmax = clkdef->pll_fmax;
    sc->pll_en_mask = clkdef->pll_en_mask;
    sc->pll_pd_reg = clkdef->pll_pd_reg;
    sc->pll_tuner_reg = clkdef->pll_tuner_reg;
    sc->pll_tuner_en_bit = clkdef->pll_tuner_en_bit;
    sc->pll_pd_shift = clkdef->pll_pd_shift;
    sc->pll_pcwbits = clkdef->pll_pcwbits;
    sc->pll_pcwibits = clkdef->pll_pcwibits;
    sc->pll_pcw_reg = clkdef->pll_pcw_reg;
    sc->pll_pcw_chg_reg = clkdef->pll_pcw_chg_reg;
    sc->pll_pcw_shift = clkdef->pll_pcw_shift;
    sc->pll_div_table = clkdef->pll_div_table;
    sc->pll_rst_bar_mask = clkdef->pll_rst_bar_mask;

    if (clknode_register(clkdom, clk) == NULL)
        return (ENXIO);

    return (0);
}