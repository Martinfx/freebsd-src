/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <dev/clk/clk.h>
#include "mt_clk_pll.h"
#include "clkdev_if.h"

#define	PLL_PWR_ON		(1u << 0)
#define	PLL_ISO_EN		(1u << 1)
#define	PLL_EN			(1u << 0)

#define	PLL_POSTDIV_MASK	0x7
#define	PLL_POSTDIV_MAX		4	/* Valid post dividers are /1 ... /16 */
#define	PLL_INTEGER_BITS	7
#define	PLL_FMIN_DEFAULT	1000000000ULL

#define	PLL_PWR_DELAY		1	/* us, PWR_ON and ISO_EN settle */
#define	PLL_LOCK_DELAY		20	/* us, PLL lock after EN */

#define	RD4(_clk, off, val)						\
	CLKDEV_READ_4(clknode_get_device(_clk), off, val)
#define	MD4(_clk, off, clr, set)					\
	CLKDEV_MODIFY_4(clknode_get_device(_clk), off, clr, set)
#define	DEVICE_LOCK(_clk)						\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)						\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

struct mt_clk_pll_sc {
	uint32_t	base_reg;
	uint32_t	pwr_reg;
	uint32_t	en_mask;
	uint32_t	rst_bar_mask;
	uint32_t	pd_reg;
	uint32_t	pd_shift;
	uint32_t	pcw_reg;
	uint32_t	pcw_shift;
	uint32_t	pcwbits;
	uint32_t	pcwibits;
	uint64_t	fmin;
	uint64_t	fmax;
	uint32_t	flags;
	bool		reported;
};

struct mt_clk_pll_state {
	uint32_t	con0;		/* Raw registers, for diagnostics */
	uint32_t	pwr;
	bool		enabled;
	uint32_t	postdiv;	/* Raw 3 bit field */
	uint32_t	pcw;
};

static int
mt_clk_pll_read_state(struct clknode *clk, struct mt_clk_pll_state *st)
{
	struct mt_clk_pll_sc *sc;
	uint32_t con0, pwr, pd, pcw;
	int rv;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(clk);
	rv = RD4(clk, sc->pwr_reg, &pwr);
	if (rv == 0)
		rv = RD4(clk, sc->base_reg, &con0);
	if (rv == 0)
		rv = RD4(clk, sc->pd_reg, &pd);
	if (rv == 0)
		rv = RD4(clk, sc->pcw_reg, &pcw);
	DEVICE_UNLOCK(clk);
	if (rv != 0)
		return (rv);

	st->con0 = con0;
	st->pwr = pwr;
	st->enabled = (con0 & PLL_EN) != 0;
	st->postdiv = (pd >> sc->pd_shift) & PLL_POSTDIV_MASK;
	st->pcw = (pcw >> sc->pcw_shift) & ((1u << sc->pcwbits) - 1);

	return (0);
}

/* Same rounding as Linux __mtk_pll_recalc_rate(). */
static uint64_t
mt_clk_pll_calc_freq(struct mt_clk_pll_sc *sc, uint64_t fin, uint32_t pcw,
    uint32_t postdiv)
{
	uint64_t vco;
	uint32_t fbits;

	fbits = sc->pcwbits > sc->pcwibits ? sc->pcwbits - sc->pcwibits : 0;
	vco = fin * pcw;
	if (fbits != 0 && (vco & ((1ULL << fbits) - 1)) != 0)
		vco = (vco >> fbits) + 1;
	else
		vco >>= fbits;

	return (howmany(vco, 1ULL << postdiv));
}

/* Check that the state programmed by the firmware makes sense. */
static int
mt_clk_pll_check(struct mt_clk_pll_sc *sc, struct mt_clk_pll_state *st)
{

	if (!st->enabled)
		return (ENXIO);
	if (st->pcw == 0 || st->postdiv > PLL_POSTDIV_MAX)
		return (EINVAL);
	return (0);
}

static int
mt_clk_pll_init(struct clknode *clk, device_t dev)
{
	struct mt_clk_pll_sc *sc;
	struct mt_clk_pll_state st;
	int rv;

	sc = clknode_get_softc(clk);
	clknode_init_parent_idx(clk, 0);

	rv = mt_clk_pll_read_state(clk, &st);
	if (rv != 0) {
		device_printf(dev, "%s: cannot read PLL registers: %d\n",
		    clknode_get_name(clk), rv);
		return (0);
	}

	rv = mt_clk_pll_check(sc, &st);
	if (bootverbose)
		device_printf(dev, "%s: CON0 %#x PWR_CON %#x%s%s%s\n",
		    clknode_get_name(clk), st.con0, st.pwr,
		    (st.pwr & PLL_PWR_ON) == 0 ? " PWR_OFF" : "",
		    (st.pwr & PLL_ISO_EN) != 0 ? " ISO" : "",
		    (sc->flags & MT_PLL_HAVE_RST_BAR) != 0 &&
		    (st.con0 & sc->rst_bar_mask) == 0 ? " RST" : "");
	if (rv == ENXIO) {
		if (bootverbose)
			device_printf(dev, "%s: disabled\n",
			    clknode_get_name(clk));
		return (0);
	}
	if (rv != 0) {
		device_printf(dev, "%s: invalid setup (pcw %#x, postdiv %u)\n",
		    clknode_get_name(clk), st.pcw, st.postdiv);
		return (0);
	}

	/*
	 * The parent is not linked yet, the frequency is checked and
	 * reported by the first recalc.
	 */
	return (0);
}

static int
mt_clk_pll_recalc(struct clknode *clk, uint64_t *freq)
{
	struct mt_clk_pll_sc *sc;
	struct mt_clk_pll_state st;
	uint64_t fin, vco;
	int rv;

	sc = clknode_get_softc(clk);

	rv = mt_clk_pll_read_state(clk, &st);
	if (rv != 0)
		return (rv);

	/*
	 * As in Linux the rate does not depend on the gate: a disabled PLL
	 * reports the frequency it will run at once enabled.  A PLL without
	 * a valid setup cannot produce any clock.
	 */
	if (st.pcw == 0 || st.postdiv > PLL_POSTDIV_MAX) {
		*freq = 0;
		return (0);
	}

	fin = *freq;
	*freq = mt_clk_pll_calc_freq(sc, fin, st.pcw, st.postdiv);

	if (!sc->reported && fin != 0) {
		sc->reported = true;
		vco = *freq << st.postdiv;
		if (vco < sc->fmin || vco > sc->fmax)
			printf("%s: VCO %ju Hz out of range %ju - %ju Hz\n",
			    clknode_get_name(clk), (uintmax_t)vco,
			    (uintmax_t)sc->fmin, (uintmax_t)sc->fmax);
		if (bootverbose)
			printf("%s: %ju Hz (pcw %#x, postdiv /%u)\n",
			    clknode_get_name(clk), (uintmax_t)*freq, st.pcw,
			    1u << st.postdiv);
	}
	return (0);
}

static int
mt_clk_pll_get_gate(struct clknode *clk, bool *enabled)
{
	struct mt_clk_pll_state st;
	int rv;

	rv = mt_clk_pll_read_state(clk, &st);
	if (rv != 0)
		return (rv);
	*enabled = st.enabled;
	return (0);
}

/* Linux mtk_pll_prepare() */
static int
mt_clk_pll_enable(struct clknode *clk, struct mt_clk_pll_sc *sc)
{
	int rv;

	rv = MD4(clk, sc->pwr_reg, 0, PLL_PWR_ON);
	if (rv != 0)
		return (rv);
	DELAY(PLL_PWR_DELAY);

	rv = MD4(clk, sc->pwr_reg, PLL_ISO_EN, 0);
	if (rv != 0)
		return (rv);
	DELAY(PLL_PWR_DELAY);

	rv = MD4(clk, sc->base_reg, 0, PLL_EN | sc->en_mask);
	if (rv != 0)
		return (rv);
	DELAY(PLL_LOCK_DELAY);

	if ((sc->flags & MT_PLL_HAVE_RST_BAR) != 0)
		rv = MD4(clk, sc->base_reg, 0, sc->rst_bar_mask);
	return (rv);
}

/* Linux mtk_pll_unprepare() */
static int
mt_clk_pll_disable(struct clknode *clk, struct mt_clk_pll_sc *sc)
{
	int rv;

	if ((sc->flags & MT_PLL_HAVE_RST_BAR) != 0) {
		rv = MD4(clk, sc->base_reg, sc->rst_bar_mask, 0);
		if (rv != 0)
			return (rv);
	}

	rv = MD4(clk, sc->base_reg, PLL_EN | sc->en_mask, 0);
	if (rv != 0)
		return (rv);

	rv = MD4(clk, sc->pwr_reg, 0, PLL_ISO_EN);
	if (rv != 0)
		return (rv);

	return (MD4(clk, sc->pwr_reg, PLL_PWR_ON, 0));
}

static int
mt_clk_pll_set_gate(struct clknode *clk, bool enable)
{
	struct mt_clk_pll_sc *sc;
	struct mt_clk_pll_state st;
	int rv;

	sc = clknode_get_softc(clk);

	/* Always-on and critical PLLs feed the CPU, buses and DRAM. */
	if (!enable && (sc->flags & (MT_PLL_AO | MT_PLL_CRITICAL)) != 0)
		return (0);

	rv = mt_clk_pll_read_state(clk, &st);
	if (rv != 0)
		return (rv);

	/* Nothing to do, and never re-run the sequence on a running PLL. */
	if (st.enabled == enable)
		return (0);

	if (enable && (st.pcw == 0 || st.postdiv > PLL_POSTDIV_MAX)) {
		printf("%s: cannot enable PLL, invalid setup "
		    "(pcw %#x, postdiv %u)\n", clknode_get_name(clk),
		    st.pcw, st.postdiv);
		return (EINVAL);
	}

	DEVICE_LOCK(clk);
	if (enable)
		rv = mt_clk_pll_enable(clk, sc);
	else
		rv = mt_clk_pll_disable(clk, sc);
	DEVICE_UNLOCK(clk);
	if (rv != 0)
		return (rv);

	/* Verify that the hardware followed. */
	rv = mt_clk_pll_read_state(clk, &st);
	if (rv != 0)
		return (rv);
	if (st.enabled != enable) {
		printf("%s: PLL %s failed (CON0 %#x PWR_CON %#x)\n",
		    clknode_get_name(clk), enable ? "enable" : "disable",
		    st.con0, st.pwr);
		return (EIO);
	}
	if (bootverbose)
		printf("%s: PLL %s\n", clknode_get_name(clk),
		    enable ? "enabled" : "disabled");
	return (0);
}

static int
mt_clk_pll_calc_values(struct mt_clk_pll_sc *sc, uint64_t fin, uint64_t freq,
    int flags, uint32_t *pcwp, uint32_t *postdivp, uint64_t *foutp)
{
	uint64_t fout, pcw, pcw_max;
	uint32_t fbits, val;

	if (fin == 0 || freq == 0)
		return (EINVAL);

	if (freq > sc->fmax) {
		if ((flags & CLK_SET_ROUND_DOWN) == 0)
			return (ERANGE);
		freq = sc->fmax;
	}

	for (val = 0; val <= PLL_POSTDIV_MAX; val++) {
		if ((freq << val) >= sc->fmin)
			break;
	}
	if (val > PLL_POSTDIV_MAX)
		return (ERANGE);

	fbits = sc->pcwbits > sc->pcwibits ? sc->pcwbits - sc->pcwibits : 0;
	pcw_max = (1ULL << sc->pcwbits) - 1;
	pcw = ((freq << val) << fbits) / fin;
	if (pcw == 0 || pcw > pcw_max)
		return (ERANGE);

	fout = mt_clk_pll_calc_freq(sc, fin, pcw, val);
	if ((flags & CLK_SET_ROUND_UP) != 0 && fout < freq && pcw < pcw_max)
		fout = mt_clk_pll_calc_freq(sc, fin, ++pcw, val);
	else if ((flags & CLK_SET_ROUND_DOWN) != 0 && fout > freq && pcw > 1)
		fout = mt_clk_pll_calc_freq(sc, fin, --pcw, val);

	if (fout != freq && CLK_SET_ROUND(flags) == 0)
		return (ERANGE);
	if ((flags & CLK_SET_ROUND_ANY) == CLK_SET_ROUND_UP && fout < freq)
		return (ERANGE);
	if ((flags & CLK_SET_ROUND_ANY) == CLK_SET_ROUND_DOWN && fout > freq)
		return (ERANGE);

	*pcwp = (uint32_t)pcw;
	*postdivp = val;
	*foutp = fout;
	return (0);
}

static int
mt_clk_pll_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
	struct mt_clk_pll_sc *sc;
	struct mt_clk_pll_state st;
	uint64_t cur, freq;
	uint32_t pcw, postdiv;
	int rv;

	sc = clknode_get_softc(clk);

	rv = mt_clk_pll_read_state(clk, &st);
	if (rv != 0)
		return (rv);

	/* The frequency the PLL runs at is always acceptable. */
	if (st.pcw != 0 && st.postdiv <= PLL_POSTDIV_MAX) {
		cur = mt_clk_pll_calc_freq(sc, fin, st.pcw, st.postdiv);
		if (*fout == cur) {
			*done = 1;
			return (0);
		}
	}

	rv = mt_clk_pll_calc_values(sc, fin, *fout, flags, &pcw, &postdiv,
	    &freq);
	if (rv != 0)
		return (rv);

	/* Dry run: only report what would be set. */
	if ((flags & CLK_SET_DRYRUN) != 0) {
		*fout = freq;
		*done = 1;
		return (0);
	}

	if (pcw == st.pcw && postdiv == st.postdiv) {
		*fout = freq;
		*done = 1;
		return (0);
	}

	/* Reprogramming a PLL is not supported yet. */
	printf("%s: changing PLL frequency to %ju Hz is not supported\n",
	    clknode_get_name(clk), (uintmax_t)freq);
	return (EOPNOTSUPP);
}

static clknode_method_t mt_clk_pll_methods[] = {
	CLKNODEMETHOD(clknode_init,		mt_clk_pll_init),
	CLKNODEMETHOD(clknode_recalc_freq,	mt_clk_pll_recalc),
	CLKNODEMETHOD(clknode_get_gate,		mt_clk_pll_get_gate),
	CLKNODEMETHOD(clknode_set_gate,		mt_clk_pll_set_gate),
	CLKNODEMETHOD(clknode_set_freq,		mt_clk_pll_set_freq),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(mt_clk_pll, mt_clk_pll_class, mt_clk_pll_methods,
    sizeof(struct mt_clk_pll_sc), clknode_class);

int
mt_clk_pll_register(struct clkdom *clkdom, struct mt_clk_pll_def *clkdef)
{
	struct clknode *clk;
	struct mt_clk_pll_sc *sc;

	if (clkdef->pcwbits == 0 || clkdef->pcwbits > 31 ||
	    clkdef->fmax == 0)
		return (EINVAL);

	clk = clknode_create(clkdom, &mt_clk_pll_class, &clkdef->clkdef);
	if (clk == NULL)
		return (ENXIO);

	sc = clknode_get_softc(clk);
	sc->base_reg = clkdef->base_reg;
	sc->pwr_reg = clkdef->pwr_reg;
	sc->en_mask = clkdef->en_mask;
	sc->rst_bar_mask = clkdef->rst_bar_mask;
	sc->pd_reg = clkdef->pd_reg;
	sc->pd_shift = clkdef->pd_shift;
	sc->pcw_reg = clkdef->pcw_reg;
	sc->pcw_shift = clkdef->pcw_shift;
	sc->pcwbits = clkdef->pcwbits;
	sc->pcwibits = clkdef->pcwibits != 0 ?
	    clkdef->pcwibits : PLL_INTEGER_BITS;
	sc->fmin = clkdef->fmin != 0 ? clkdef->fmin : PLL_FMIN_DEFAULT;
	sc->fmax = clkdef->fmax;
	sc->flags = clkdef->flags;

	clknode_register(clkdom, clk);
	return (0);
}
