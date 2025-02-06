#ifndef __MT7622_CLK_H__
#define __MT7622_CLK_H__

/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Martin Filla, Michal Meloun
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */


#include <dev/clk/clk.h>
#include <dev/clk/clk_div.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_mux.h>
#include <dev/clk/clk_gate.h>

/* Parent list */
#define PLIST(_name) static const char *_name[]

/* Standard gate. */
#define	GATE(_id, cname, plist, o, s)					\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cname,						\
	.clkdef.parent_names = (const char *[]){plist},			\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.offset = o,							\
	.shift = s,							\
	.mask = 1,							\
	.on_value = 1,							\
	.off_value = 0,							\
}

/* Fixed rate clock. */
#define	FRATE(_id, _name, _freq)					\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_cnt = 0,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.freq = _freq,							\
}

/* Link */
#define	LINK(_idx, _clkname, _pname)				\
{									\
    .clkdef.id = _idx,							\
    .clkdef.name = _clkname,						\
    .clkdef.parent_name = _pname,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS \
}

/* Fixed factor clock */
#define	FFACT(_id, _name, _pname, _mult, _div)				\
{									\
	.clkdef.id = _id,					\
	.clkdef.name = _name,					\
	.clkdef.parent_names = (const char *[]){_pname},	\
	.clkdef.parent_cnt = 1,					\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,		\
	.mult = _mult,						\
	.div = _div,						\
}

/* Divided clock */
#define DIV(_id, _name, _pname, _reg, _shift, _width) {	\
		.clkdef.id = _id,					\
		.clkdef.name = _name,					\
		.clkdef.parent_names = (const char *[]){_pname},				\
		.offset = _reg,				\
		.i_shift = _shift \
		.i_width = _width \
}

/* Pure multiplexer. */
#define	MUX0(_id, cname, plists, _reg, _shift, _width)				\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = cname,						\
	.clkdef.parent_names = plists,					\
	.clkdef.parent_cnt = nitems(plists),				\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.offset = _reg,							\
	.shift  = _shift,							\
	.width = _width,							\
}

/* Full composite clock. */
#define COMP(_name, _parents, _mux_reg, _mux_shift, _mux_width, \
                            _divider_reg, _divider_shift, _divider_width,      \
                            _gate_reg, _gate_shift, _flags, _mux_flags)        \
{                                                                          \
	.type = MK_CLK_COMPOSITE, \
    .name = _name,                                                         \
    .parent_names = _parents,                                              \
    .num_parents = nitems(_parents),                                       \
    .mux_reg = _mux_reg,                                                   \
    .divider_reg = _divider_reg,                                           \
    .gate_reg = _gate_reg,                                                 \
    .mux_shift = _mux_shift,                                               \
    .mux_width = _mux_width,                                               \
    .divider_shift = _divider_shift,                                       \
    .divider_width = _divider_width,                                       \
    .gate_shift = _gate_shift,                                             \
    .flags = _flags,                                                       \
    .mux_flags = _mux_flags,                                               \
}

struct mdtk_clk_softc {
	device_t		dev;
	struct resource *	mem_res;
	struct mtx		mtx;
	struct clkdom 		*clkdom;
	int			type;
};

#endif
