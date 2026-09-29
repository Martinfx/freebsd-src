/*
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef __MT_CLK_PLL_H__
#define __MT_CLK_PLL_H__

#include <dev/clk/clk.h>

/*
 * Mediatek PLL.
 *
 * The output frequency is given by the PCW (feedback divider, 7 integer
 * bits followed by pcwbits - 7 fractional bits) and a 3 bit power-of-two
 * post divider:
 *
 *	fout = fin * pcw / 2^(pcwbits - 7) / 2^postdiv
 */
struct mt_clk_pll_def {
	struct clknode_init_def	clkdef;
	uint32_t		base_reg;	/* CON0, EN is bit 0 */
	uint32_t		pwr_reg;	/* PWR_CON0 */
	uint32_t		en_mask;	/* Extra enable bits in CON0 */
	uint32_t		rst_bar_mask;	/* CON0 reset bar */
	uint32_t		pd_reg;		/* Post divider register */
	uint32_t		pd_shift;
	uint32_t		pcw_reg;	/* PCW register */
	uint32_t		pcw_shift;
	uint32_t		pcwbits;	/* Total PCW width */
	uint32_t		pcwibits;	/* Integer PCW bits, 0 = 7 */
	uint64_t		fmin;		/* VCO range, 0 = 1000 MHz */
	uint64_t		fmax;
	uint32_t		flags;
#define	MT_PLL_HAVE_RST_BAR	0x0001
#define	MT_PLL_AO		0x0002	/* Always on, never disable. */
#define	MT_PLL_CRITICAL		0x0004	/* Never disable, system depends on it. */
};

#define	MT_PLL(_id, _name, _pname, _reg, _pwr_reg, _en_mask, _flags,	\
    _pcwbits, _pd_reg, _pd_shift, _pcw_reg, _pcw_shift, _rst_bar, _fmax) \
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = (const char *[]){_pname},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.base_reg = _reg,						\
	.pwr_reg = _pwr_reg,						\
	.en_mask = _en_mask,						\
	.rst_bar_mask = _rst_bar,					\
	.pd_reg = _pd_reg,						\
	.pd_shift = _pd_shift,						\
	.pcw_reg = _pcw_reg,						\
	.pcw_shift = _pcw_shift,					\
	.pcwbits = _pcwbits,						\
	.fmax = _fmax,							\
	.flags = _flags,						\
}

int mt_clk_pll_register(struct clkdom *clkdom, struct mt_clk_pll_def *clkdef);

#endif /* __MT_CLK_PLL_H__ */
