/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <dev/clk/clk.h>
#include "mt_clk_cpumux.h"
#include "clkdev_if.h"

#define	RD4(_clk, off, val)						\
	CLKDEV_READ_4(clknode_get_device(_clk), off, val)
#define	MD4(_clk, off, clr, set)					\
	CLKDEV_MODIFY_4(clknode_get_device(_clk), off, clr, set)
#define	DEVICE_LOCK(_clk)						\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)						\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

struct mt_clk_cpumux_sc {
	uint32_t	offset;
	uint32_t	shift;
	uint32_t	mask;
	int		main_parent;
	int		alt_parent;
	bool		alt_enabled;
};

static int
mt_clk_cpumux_init(struct clknode *clk, device_t dev)
{
	struct mt_clk_cpumux_sc *sc;
	uint32_t reg;
	int rv;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(clk);
	rv = RD4(clk, sc->offset, &reg);
	DEVICE_UNLOCK(clk);
	if (rv != 0)
		return (rv);

	clknode_init_parent_idx(clk, (reg >> sc->shift) & sc->mask);
	return (0);
}

static int
mt_clk_cpumux_set_mux(struct clknode *clk, int idx)
{
	struct mt_clk_cpumux_sc *sc;
	int rv;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(clk);
	rv = MD4(clk, sc->offset, sc->mask << sc->shift,
	    ((uint32_t)idx & sc->mask) << sc->shift);
	DEVICE_UNLOCK(clk);
	return (rv);
}

/* Linux mtk_cpufreq_set_target() */
static int
mt_clk_cpumux_set_freq(struct clknode *clk, uint64_t fin, uint64_t *fout,
    int flags, int *done)
{
	struct mt_clk_cpumux_sc *sc;
	struct clknode *p_main, *p_alt;
	const char **p_names;
	uint64_t freq;
	int rv, rv2;

	sc = clknode_get_softc(clk);
	p_names = clknode_get_parent_names(clk);
	p_main = clknode_find_by_name(p_names[sc->main_parent]);
	p_alt = clknode_find_by_name(p_names[sc->alt_parent]);
	if (p_main == NULL || p_alt == NULL)
		return (ENXIO);

	/* Dry run: only ask the PLL. */
	if ((flags & CLK_SET_DRYRUN) != 0) {
		rv = clknode_test_freq(p_main, *fout, flags, 0, &freq);
		if (rv != 0)
			return (rv);
		*fout = freq;
		*done = 1;
		return (0);
	}

	/* The intermediate clock must run before the CPU is moved to it. */
	if (!sc->alt_enabled) {
		rv = clknode_enable(p_alt);
		if (rv != 0) {
			printf("%s: cannot enable %s: %d\n",
			    clknode_get_name(clk), clknode_get_name(p_alt), rv);
			return (rv);
		}
		sc->alt_enabled = true;
	}

	/* Keep the CPU running from the intermediate clock meanwhile. */
	rv = clknode_set_parent_by_idx(clk, sc->alt_parent);
	if (rv != 0) {
		printf("%s: cannot switch to %s: %d\n",
		    clknode_get_name(clk), clknode_get_name(p_alt), rv);
		return (rv);
	}

	rv = clknode_set_freq(p_main, *fout, flags, 1);
	if (rv != 0)
		printf("%s: cannot set %s to %ju Hz: %d\n",
		    clknode_get_name(clk), clknode_get_name(p_main),
		    (uintmax_t)*fout, rv);

	/* Always go back to the PLL, also when the change failed. */
	rv2 = clknode_set_parent_by_idx(clk, sc->main_parent);
	if (rv2 != 0) {
		printf("%s: cannot switch back to %s: %d\n",
		    clknode_get_name(clk), clknode_get_name(p_main), rv2);
		if (rv == 0)
			rv = rv2;
	}
	if (rv != 0)
		return (rv);

	rv = clknode_get_freq(p_main, &freq);
	if (rv != 0)
		return (rv);

	*fout = freq;
	*done = 1;
	return (0);
}

static clknode_method_t mt_clk_cpumux_methods[] = {
	CLKNODEMETHOD(clknode_init,		mt_clk_cpumux_init),
	CLKNODEMETHOD(clknode_set_mux,		mt_clk_cpumux_set_mux),
	CLKNODEMETHOD(clknode_set_freq,		mt_clk_cpumux_set_freq),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(mt_clk_cpumux, mt_clk_cpumux_class, mt_clk_cpumux_methods,
    sizeof(struct mt_clk_cpumux_sc), clknode_class);

int
mt_clk_cpumux_register(struct clkdom *clkdom, struct mt_clk_cpumux_def *clkdef)
{
	struct clknode *clk;
	struct mt_clk_cpumux_sc *sc;

	if (clkdef->width == 0 || clkdef->width > 31 ||
	    clkdef->main_parent >= clkdef->clkdef.parent_cnt ||
	    clkdef->alt_parent >= clkdef->clkdef.parent_cnt)
		return (EINVAL);

	clk = clknode_create(clkdom, &mt_clk_cpumux_class, &clkdef->clkdef);
	if (clk == NULL)
		return (ENXIO);

	sc = clknode_get_softc(clk);
	sc->offset = clkdef->offset;
	sc->shift = clkdef->shift;
	sc->mask = (1u << clkdef->width) - 1;
	sc->main_parent = clkdef->main_parent;
	sc->alt_parent = clkdef->alt_parent;

	clknode_register(clkdom, clk);
	return (0);
}
