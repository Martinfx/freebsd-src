#ifndef __MDTK_CLK_H__
#define __MDTK_CLK_H__

/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2025 Martin Filla
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

#include <arm64/mediatek/mdtk_clk_gate.h>
/* Parent list */
#define PLIST(_name) static const char *_name[]

/* Pure gate */
#define	GATE(_idx, _clkname, _pname, _o, _s)				\
{									\
    .id = _idx,							\
    .name = _clkname,						\
    .parent_name = _pname,						\
    .offset = CRU_CLKGATE_CON(_o),					\
    .shift = _s,							\
}

/* Fixed rate clock. */
#define	FRATE(_id, _name, _freq)					\
{									\
    .type = MK_CLK_FIXED,						\
    .clk.fixed = &(struct clk_fixed_def) {				\
        .clkdef.id = _id,					\
        .clkdef.name = _name,					\
        .clkdef.parent_names = NULL,				\
        .clkdef.parent_cnt = 0,					\
        .clkdef.flags = CLK_NODE_STATIC_STRINGS,		\
        .freq = _freq,						\
    },								\
}

/* Fixed factor clock */
#define FACTOR(_name, _parent, _mult, _div) {	\
        .type = MK_CLK_FACTOR, \
        .name = _name,		\
        .parent_name = _parent,	\
        .mult = _mult,		\
        .div = _div,		\
    }

#define MDTK_COMPOSITE_CLK(_name, _parents, _mux_reg, _mux_shift, _mux_width, \
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

struct mdtk_clk_gate {
	const char	*name;
	const char	*parent_name;
	uint32_t	id;
	uint32_t	offset;
	uint32_t	shift;
};

enum mdtk_clk_type {
	MK_CLK_UNDEFINED = 0,
    MK_CLK_PLL,
    MK_CLK_COMPOSITE,
    MK_CLK_FIXED,
    MK_CLK_FRACT,
    MK_CLK_MUX,
    MK_CLK_LINK,
    MK_CLK_FACTOR
};

struct mdtk_fixed_factor {
    const char *name;
    const char *parent_name;
    int mult;
    int div;
};

struct mdtk_composite_clk {
    const char *name;
    const char * const *parent_names;
    int num_parents;
    uint32_t mux_reg;
    uint32_t divider_reg;
    uint32_t gate_reg;
    int mux_shift;
    int mux_width;
    int gate_shift;
    int divider_shift;
    int divider_width;
    uint8_t mux_flags;
    unsigned int flags;
};


struct mdtk_clk {
	enum mdtk_clk_type	type;
	union {
		struct rk_clk_composite_def	*composite;
		struct clk_fixed_def		*fixed;
		struct clk_link_def		*link;
	} clk;
};

struct mdtk_clk_softc {
    device_t		dev;
    struct resource		*res;
    struct clkdom		*clkdom;
    struct mtx		mtx;
    int			type;
    uint32_t		reset_offset;
    uint32_t		reset_num;
	struct mdtk_clk_gate *gates;
	int			ngates;
	struct mdtk_clk		*clks;
	int			nclks;
};

DECLARE_CLASS(mdtk_clk_driver);

int	mdtk_attach(device_t dev);

#endif
