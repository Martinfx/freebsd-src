/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

/*
 * MediaTek PLL clock node, with the MT7622 register layout.  CON0 holds the
 * enable bit.  The post divider (a 3-bit field holding a power of two up to
 * 16) and the frequency control word (PCW) live in the register after CON0
 * or in CON0 itself, depending on the PLL.  The PCW has 7 integer bits
 * followed by pcwbits - 7 fractional bits:
 *
 *	rate = parent * PCW / 2^(pcwbits - 7) / 2^postdiv
 *
 * rounded up as the Linux driver does, so the rates match.  A new PCW is
 * latched by setting bit 31 of the register after CON0, while the PLL runs.
 *
 * Firmware starts the PLLs and the node has no gate: a PLL that is off
 * reports 0 Hz and its rate cannot be set.  The rate of a PLL that may
 * never stop is not changed either, it feeds clocks that do not expect it.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <dev/clk/clk.h>
#include <dev/clk/mediatek/mtk_clk.h>

#include "clkdev_if.h"

#define	PLL_CON0_EN		0x00000001
#define	PLL_CON1		0x4	/* from CON0 */
#define	PLL_PCW_CHG		0x80000000	/* in CON1 */
#define	PLL_POSTDIV_MASK	0x7
#define	PLL_POSTDIV_MAX		4	/* divide by 16 */
#define	PLL_PCW_IBITS		7	/* integer bits of the PCW */

/* VCO range of the Linux driver. */
#define	PLL_VCO_MIN		1000000000ULL
#define	PLL_VCO_MAX		2500000000ULL

#define	PLL_LOCK_DELAY		20	/* us, after a PCW change */

struct mt7622_clk_pll_sc {
	uint32_t	con0;
	uint32_t	pd_reg;
	uint32_t	pd_shift;
	uint32_t	pcw_reg;
	uint32_t	pcw_shift;
	uint32_t	pcw_mask;	/* unshifted */
	uint32_t	pcw_fbits;
};

#define	RD4(_clk, _off, _val)						\
	CLKDEV_READ_4(clknode_get_device(_clk), _off, _val)
#define	MD4(_clk, _off, _clr, _set)					\
	CLKDEV_MODIFY_4(clknode_get_device(_clk), _off, _clr, _set)
#define	DEVICE_LOCK(_clk)						\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)						\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

static int
mt7622_clk_pll_init(struct clknode *clk, device_t dev)
{
	clknode_init_parent_idx(clk, 0);
	return (0);
}

/* Rate for a PCW and post divider, as Linux __mtk_pll_recalc_rate(). */
static uint64_t
mt7622_clk_pll_rate(struct mt7622_clk_pll_sc *sc, uint64_t fin, uint64_t pcw,
    uint32_t postdiv)
{
	uint64_t vco;

	vco = fin * pcw;
	if (sc->pcw_fbits != 0)
		vco = howmany(vco, 1ULL << sc->pcw_fbits);
	return (howmany(vco, 1ULL << postdiv));
}

static int
mt7622_clk_pll_recalc_freq(struct clknode *clk, uint64_t *freq)
{
	struct mt7622_clk_pll_sc *sc = clknode_get_softc(clk);
	uint32_t con0, pd, pcw;

	DEVICE_LOCK(clk);
	RD4(clk, sc->con0, &con0);
	RD4(clk, sc->pd_reg, &pd);
	RD4(clk, sc->pcw_reg, &pcw);
	DEVICE_UNLOCK(clk);

	if ((con0 & PLL_CON0_EN) == 0)
		*freq = 0;
	else
		*freq = mt7622_clk_pll_rate(sc, *freq,
		    (pcw >> sc->pcw_shift) & sc->pcw_mask,
		    (pd >> sc->pd_shift) & PLL_POSTDIV_MASK);
	return (0);
}

/* Set a new PCW and post divider, as Linux mtk_pll_set_rate_regs(). */
static void
mt7622_clk_pll_program(struct clknode *clk, uint32_t pcw, uint32_t postdiv)
{
	struct mt7622_clk_pll_sc *sc = clknode_get_softc(clk);
	uint32_t pcw_mask, pd_mask;

	pcw_mask = sc->pcw_mask << sc->pcw_shift;
	pd_mask = PLL_POSTDIV_MASK << sc->pd_shift;

	DEVICE_LOCK(clk);
	/* Post divider and PCW are written at once if they share a register. */
	if (sc->pd_reg == sc->pcw_reg) {
		MD4(clk, sc->pcw_reg, pcw_mask | pd_mask,
		    (pcw << sc->pcw_shift) | (postdiv << sc->pd_shift));
	} else {
		MD4(clk, sc->pd_reg, pd_mask, postdiv << sc->pd_shift);
		MD4(clk, sc->pcw_reg, pcw_mask, pcw << sc->pcw_shift);
	}
	MD4(clk, sc->con0 + PLL_CON1, 0, PLL_PCW_CHG);
	DELAY(PLL_LOCK_DELAY);
	DEVICE_UNLOCK(clk);
}

/*
 * Use the smallest post divider that keeps the VCO in its range, as the
 * Linux driver does.  The rounding only chooses between two neighbouring
 * PCW values.
 */
static int
mt7622_clk_pll_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *stop)
{
	struct mt7622_clk_pll_sc *sc = clknode_get_softc(clk);
	uint64_t cur, freq, pcw, vco;
	uint32_t postdiv;
	int round;

	*stop = 1;
	round = CLK_SET_ROUND(flags);

	/* A PLL that is off reports 0 Hz and its rate cannot be set. */
	cur = fin;
	mt7622_clk_pll_recalc_freq(clk, &cur);
	if (cur == 0)
		return (ENXIO);

	/* A request beyond the range can only be rounded to its limit. */
	vco = *fout;
	if (vco > PLL_VCO_MAX && (round & CLK_SET_ROUND_DOWN) != 0)
		vco = PLL_VCO_MAX;
	else if (vco < (PLL_VCO_MIN >> PLL_POSTDIV_MAX) &&
	    (round & CLK_SET_ROUND_UP) != 0)
		vco = PLL_VCO_MIN >> PLL_POSTDIV_MAX;
	for (postdiv = 0; postdiv < PLL_POSTDIV_MAX && vco < PLL_VCO_MIN;
	    postdiv++)
		vco <<= 1;
	if (vco < PLL_VCO_MIN || vco > PLL_VCO_MAX)
		return (ERANGE);

	/* Cannot overflow: the VCO rate is below 2^32. */
	pcw = ((vco << sc->pcw_fbits) + fin / 2) / fin;
	freq = mt7622_clk_pll_rate(sc, fin, pcw, postdiv);
	if (round == CLK_SET_ROUND_UP && freq < *fout)
		freq = mt7622_clk_pll_rate(sc, fin, ++pcw, postdiv);
	else if (round == CLK_SET_ROUND_DOWN && freq > *fout)
		freq = mt7622_clk_pll_rate(sc, fin, --pcw, postdiv);
	else if (round == CLK_SET_ROUND_EXACT && freq != *fout)
		return (ERANGE);
	if (pcw == 0 || pcw > sc->pcw_mask)
		return (ERANGE);

	if (cur != freq) {
		if ((clknode_get_flags(clk) & CLK_NODE_CANNOT_STOP) != 0)
			return (EBUSY);
		if ((flags & CLK_SET_DRYRUN) == 0)
			mt7622_clk_pll_program(clk, (uint32_t)pcw, postdiv);
	}
	*fout = freq;
	return (0);
}

static clknode_method_t mt7622_clk_pll_methods[] = {
	CLKNODEMETHOD(clknode_init,		mt7622_clk_pll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	mt7622_clk_pll_recalc_freq),
	CLKNODEMETHOD(clknode_set_freq,		mt7622_clk_pll_set_freq),
	CLKNODEMETHOD_END
};

DEFINE_CLASS_1(mt7622_clk_pll_clknode, mt7622_clk_pll_clknode_class,
    mt7622_clk_pll_methods, sizeof(struct mt7622_clk_pll_sc), clknode_class);

int
mt7622_clk_pll_register(struct clkdom *clkdom,
    const struct mt7622_clk_pll_def *def)
{
	struct clknode *clk;
	struct mt7622_clk_pll_sc *sc;

	if (def->pcwbits < PLL_PCW_IBITS || def->pcwbits > 31)
		return (EINVAL);

	clk = clknode_create(clkdom, &mt7622_clk_pll_clknode_class,
	    &def->clkdef);
	if (clk == NULL)
		return (ENXIO);

	sc = clknode_get_softc(clk);
	sc->con0 = def->con0;
	sc->pd_reg = def->pd_reg;
	sc->pd_shift = def->pd_shift;
	sc->pcw_reg = def->pcw_reg;
	sc->pcw_shift = def->pcw_shift;
	sc->pcw_mask = (1u << def->pcwbits) - 1;
	sc->pcw_fbits = def->pcwbits - PLL_PCW_IBITS;

	if (clknode_register(clkdom, clk) == NULL)
		return (ENXIO);
	return (0);
}
