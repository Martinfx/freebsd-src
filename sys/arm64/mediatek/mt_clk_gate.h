/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Martin Filla
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

#ifndef __MT_CLK_GATE_H__
#define	__MT_CLK_GATE_H__

#include <dev/clk/clk.h>

/*
 * MediaTek clock gate with separate SET, CLR and STA registers.
 *
 * In the normal polarity a set bit in STA means "gated": the clock is
 * enabled by writing the bit to CLR and disabled by writing it to SET.
 * MT_GATE_INV swaps the meaning.  MT_GATE_NO_SETCLR is for gates that only
 * have a status register, which is then modified in place.
 */
struct mt_clk_gate_def {
	struct clknode_init_def	clkdef;
	uint32_t		set_ofs;
	uint32_t		clr_ofs;
	uint32_t		sta_ofs;
	uint32_t		shift;
	uint32_t		flags;
#define	MT_GATE_INV		0x0001	/* Bit set means enabled. */
#define	MT_GATE_NO_SETCLR	0x0002	/* No SET/CLR, modify STA. */
};

#define	MT_GATE(_id, _name, _pname, _set, _clr, _sta, _shift, _flags)	\
{									\
	.clkdef.id = _id,						\
	.clkdef.name = _name,						\
	.clkdef.parent_names = (const char *[]){_pname},		\
	.clkdef.parent_cnt = 1,						\
	.clkdef.flags = CLK_NODE_STATIC_STRINGS,			\
	.set_ofs = _set,						\
	.clr_ofs = _clr,						\
	.sta_ofs = _sta,						\
	.shift = _shift,						\
	.flags = _flags,						\
}

int mt_clk_gate_register(struct clkdom *clkdom,
    struct mt_clk_gate_def *clkdef);

#endif /* __MT_CLK_GATE_H__ */
