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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>

#include <dev/clk/clk.h>

#include <arm64/mediatek/mt_clk_gate.h>

#include "clkdev_if.h"

#define	WR4(_clk, off, val)						\
	CLKDEV_WRITE_4(clknode_get_device(_clk), off, val)
#define	RD4(_clk, off, val)						\
	CLKDEV_READ_4(clknode_get_device(_clk), off, val)
#define	MD4(_clk, off, clr, set)					\
	CLKDEV_MODIFY_4(clknode_get_device(_clk), off, clr, set)
#define	DEVICE_LOCK(_clk)						\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)						\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

struct mt_clk_gate_sc {
	uint32_t	set_ofs;
	uint32_t	clr_ofs;
	uint32_t	sta_ofs;
	uint32_t	mask;
	uint32_t	flags;
};

static int
mt_clk_gate_init(struct clknode *clk, device_t dev)
{

	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
mt_clk_gate_set_gate(struct clknode *clk, bool enable)
{
	struct mt_clk_gate_sc *sc;
	bool setbit;
	int rv;

	sc = clknode_get_softc(clk);

	/* The bit is set to gate the clock, unless the gate is inverted. */
	setbit = (sc->flags & MT_GATE_INV) != 0 ? enable : !enable;

	DEVICE_LOCK(clk);
	if ((sc->flags & MT_GATE_NO_SETCLR) != 0)
		rv = MD4(clk, sc->sta_ofs, sc->mask, setbit ? sc->mask : 0);
	else
		rv = WR4(clk, setbit ? sc->set_ofs : sc->clr_ofs, sc->mask);
	DEVICE_UNLOCK(clk);

	return (rv);
}

static int
mt_clk_gate_get_gate(struct clknode *clk, bool *enabled)
{
	struct mt_clk_gate_sc *sc;
	uint32_t reg;
	int rv;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(clk);
	rv = RD4(clk, sc->sta_ofs, &reg);
	DEVICE_UNLOCK(clk);
	if (rv != 0)
		return (rv);

	*enabled = ((reg & sc->mask) != 0) == ((sc->flags & MT_GATE_INV) != 0);
	return (0);
}

static clknode_method_t mt_clk_gate_methods[] = {
	CLKNODEMETHOD(clknode_init,	mt_clk_gate_init),
	CLKNODEMETHOD(clknode_set_gate,	mt_clk_gate_set_gate),
	CLKNODEMETHOD(clknode_get_gate,	mt_clk_gate_get_gate),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(mt_clk_gate, mt_clk_gate_class, mt_clk_gate_methods,
    sizeof(struct mt_clk_gate_sc), clknode_class);

int
mt_clk_gate_register(struct clkdom *clkdom, struct mt_clk_gate_def *clkdef)
{
	struct clknode *clk;
	struct mt_clk_gate_sc *sc;

	clk = clknode_create(clkdom, &mt_clk_gate_class, &clkdef->clkdef);
	if (clk == NULL)
		return (ENXIO);

	sc = clknode_get_softc(clk);
	sc->set_ofs = clkdef->set_ofs;
	sc->clr_ofs = clkdef->clr_ofs;
	sc->sta_ofs = clkdef->sta_ofs;
	sc->mask = 1u << clkdef->shift;
	sc->flags = clkdef->flags;

	clknode_register(clkdom, clk);
	return (0);
}
