/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef __MT_CLK_CPUMUX_H__
#define __MT_CLK_CPUMUX_H__

#include <dev/clk/clk.h>

/*
 * Mediatek CPU clock mux.
 *
 * A frequency request is passed to the main parent PLL.  While the PLL is
 * reprogrammed the CPU runs from an intermediate clock, as Linux
 * mediatek-cpufreq does.
 */
struct mt_clk_cpumux_def {
	struct clknode_init_def	clkdef;
	uint32_t		offset;
	uint32_t		shift;
	uint32_t		width;
	int			main_parent;	/* PLL index */
	int			alt_parent;	/* Intermediate clock index */
};

#define	MT_CPUMUX(_id, _name, _plist, _reg, _shift, _width, _main, _alt) \
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = _plist,					\
	.clkdef.parent_cnt = nitems(_plist),				\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.offset = _reg,							\
	.shift = _shift,						\
	.width = _width,						\
	.main_parent = _main,						\
	.alt_parent = _alt,						\
}

int mt_clk_cpumux_register(struct clkdom *clkdom,
    struct mt_clk_cpumux_def *clkdef);

#endif /* __MT_CLK_CPUMUX_H__ */
