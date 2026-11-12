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

/*
 * Pin controller and GPIO driver for the MediaTek MT8167.
 *
 * The pin registers do not live in the pinctrl node itself (that one holds
 * the external interrupt controller) but in the "syscfg_pctl" syscon the
 * "mediatek,pctl-regmap" property points to.  Most of them come as a
 * triplet: the value at +0, write one to set at +4 and write one to clear
 * at +8, sixteen pins per register.
 *
 * The external interrupt (EINT) part is not implemented yet.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <machine/bus.h>

#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/fdt/fdt_pinctrl.h>
#include <dev/gpio/gpiobusvar.h>

#include <dt-bindings/pinctrl/mt65xx.h>

#include "gpio_if.h"

#define	MT8167_NPINS		125

/* Register blocks, relative to the syscfg_pctl base. */
#define	PCTL_DIR		0x000
#define	PCTL_DOUT		0x100
#define	PCTL_DIN		0x200
#define	PCTL_MODE		0x300
#define	PCTL_PULLEN		0x500
#define	PCTL_PULLSEL		0x600

#define	PCTL_SET		0x4	/* Write one to set. */
#define	PCTL_CLR		0x8	/* Write one to clear. */

/* Sixteen pins per register, registers 0x10 apart. */
#define	PCTL_PORT(_pin)		(((_pin) >> 4) << 4)
#define	PCTL_BIT(_pin)		(1u << ((_pin) & 0xf))

/* Five pins per mode register, three bits each. */
#define	PCTL_MODE_PINS		5
#define	PCTL_MODE_BITS		3
#define	PCTL_MODE_MASK		0x7u
#define	PCTL_MODE_GPIO		0

struct mt8167_pinctrl_softc {
	device_t		dev;
	device_t		busdev;
	struct mtx		mtx;
	bus_space_tag_t		bst;
	bus_space_handle_t	bsh;
	bus_size_t		size;
};

#define	PCTL_LOCK(_sc)		mtx_lock_spin(&(_sc)->mtx)
#define	PCTL_UNLOCK(_sc)	mtx_unlock_spin(&(_sc)->mtx)

#define	RD4(_sc, _reg)							\
	bus_space_read_4((_sc)->bst, (_sc)->bsh, (_reg))
#define	WR4(_sc, _reg, _val)						\
	bus_space_write_4((_sc)->bst, (_sc)->bsh, (_reg), (_val))

/* Drive strength classes: minimum, maximum, step in mA and the bit field. */
struct mt8167_drv_class {
	uint8_t		min;
	uint8_t		max;
	uint8_t		step;
	uint8_t		low_bit;
	uint8_t		high_bit;
};

static const struct mt8167_drv_class mt8167_drv_class[] = {
	{ 4, 16, 4, 1, 2 },	/* 4/8/12/16 mA */
	{ 2, 8, 2, 1, 2 },	/* 2/4/6/8 mA */
	{ 2, 16, 2, 0, 2 },	/* 2/4/6/8/10/12/14/16 mA */
};

/* Per-pin drive strength field: pin, register, bit, class. */
struct mt8167_pin_drv {
	uint8_t		pin;
	uint16_t	reg;
	uint8_t		bit;
	uint8_t		class;
};

static const struct mt8167_pin_drv mt8167_pin_drv[] = {
	{ 0, 0xd00, 0, 0 }, { 1, 0xd00, 0, 0 }, { 2, 0xd00, 0, 0 },
	{ 3, 0xd00, 0, 0 }, { 4, 0xd00, 0, 0 }, { 5, 0xd00, 4, 0 },
	{ 6, 0xd00, 4, 0 }, { 7, 0xd00, 4, 0 }, { 8, 0xd00, 4, 0 },
	{ 9, 0xd00, 4, 0 }, { 10, 0xd00, 4, 0 }, { 11, 0xd00, 8, 0 },
	{ 12, 0xd00, 8, 0 }, { 13, 0xd00, 8, 0 }, { 14, 0xd00, 12, 2 },
	{ 15, 0xd00, 12, 2 }, { 16, 0xd00, 12, 2 }, { 17, 0xd00, 12, 2 },
	{ 18, 0xd10, 0, 0 }, { 19, 0xd10, 0, 0 }, { 20, 0xd10, 0, 0 },
	{ 21, 0xd00, 12, 2 }, { 22, 0xd00, 12, 2 }, { 23, 0xd00, 12, 2 },
	{ 24, 0xd00, 8, 0 }, { 25, 0xd00, 8, 0 }, { 26, 0xd10, 4, 1 },
	{ 27, 0xd10, 4, 1 }, { 28, 0xd10, 4, 1 }, { 29, 0xd10, 4, 1 },
	{ 30, 0xd10, 4, 1 }, { 31, 0xd10, 8, 1 }, { 32, 0xd10, 8, 1 },
	{ 33, 0xd10, 8, 1 }, { 34, 0xd10, 12, 0 }, { 35, 0xd10, 12, 0 },
	{ 36, 0xd20, 0, 0 }, { 37, 0xd20, 0, 0 }, { 38, 0xd20, 0, 0 },
	{ 39, 0xd20, 0, 0 }, { 40, 0xd20, 4, 1 }, { 41, 0xd20, 8, 1 },
	{ 42, 0xd20, 8, 1 }, { 43, 0xd20, 8, 1 }, { 44, 0xd20, 12, 1 },
	{ 45, 0xd20, 12, 1 }, { 46, 0xd20, 12, 1 }, { 47, 0xd20, 12, 1 },
	{ 48, 0xd30, 0, 1 }, { 49, 0xd30, 0, 1 }, { 50, 0xd30, 0, 1 },
	{ 51, 0xd30, 0, 1 }, { 54, 0xd30, 8, 1 }, { 55, 0xd30, 12, 1 },
	{ 56, 0xd30, 12, 1 }, { 57, 0xd30, 12, 1 }, { 62, 0xd40, 8, 1 },
	{ 63, 0xd40, 8, 1 }, { 64, 0xd40, 8, 1 }, { 65, 0xd40, 8, 1 },
	{ 66, 0xd40, 8, 1 }, { 67, 0xd40, 8, 1 }, { 68, 0xd40, 12, 2 },
	{ 69, 0xd50, 0, 2 }, { 70, 0xd50, 4, 2 }, { 71, 0xd50, 4, 2 },
	{ 72, 0xd50, 4, 2 }, { 73, 0xd50, 4, 2 }, { 100, 0xd50, 8, 1 },
	{ 101, 0xd50, 8, 1 }, { 102, 0xd50, 8, 1 }, { 103, 0xd50, 8, 1 },
	{ 104, 0xd50, 12, 2 }, { 105, 0xd60, 0, 2 }, { 106, 0xd60, 4, 2 },
	{ 107, 0xd60, 4, 2 }, { 108, 0xd60, 4, 2 }, { 109, 0xd60, 4, 2 },
	{ 110, 0xd70, 0, 2 }, { 111, 0xd70, 0, 2 }, { 112, 0xd70, 0, 2 },
	{ 113, 0xd70, 0, 2 }, { 114, 0xd70, 4, 2 }, { 115, 0xd60, 12, 2 },
	{ 116, 0xd60, 8, 2 }, { 117, 0xd70, 0, 2 }, { 118, 0xd70, 0, 2 },
	{ 119, 0xd70, 0, 2 }, { 120, 0xd70, 0, 2 },
};

/*
 * Pins with a dedicated pull register: pin, register, PUPD bit and the bits
 * of the R1 and R0 resistor selects.  PUPD set means pull down.
 */
struct mt8167_pin_pupd {
	uint8_t		pin;
	uint16_t	reg;
	uint8_t		pupd_bit;
	uint8_t		r1_bit;
	uint8_t		r0_bit;
};

static const struct mt8167_pin_pupd mt8167_pin_pupd[] = {
	{ 14, 0xe50, 14, 13, 12 }, { 15, 0xe60, 2, 1, 0 },
	{ 16, 0xe60, 6, 5, 4 }, { 17, 0xe60, 10, 9, 8 },
	{ 21, 0xe60, 14, 13, 12 }, { 22, 0xe70, 2, 1, 0 },
	{ 23, 0xe70, 6, 5, 4 }, { 40, 0xe80, 2, 1, 0 },
	{ 41, 0xe80, 6, 5, 4 }, { 42, 0xe90, 2, 1, 0 },
	{ 43, 0xe90, 6, 5, 4 }, { 68, 0xe50, 10, 9, 8 },
	{ 69, 0xe50, 6, 5, 4 }, { 70, 0xe40, 6, 5, 4 },
	{ 71, 0xe40, 10, 9, 8 }, { 72, 0xe40, 14, 13, 12 },
	{ 73, 0xe50, 2, 1, 0 }, { 104, 0xe40, 2, 1, 0 },
	{ 105, 0xe30, 14, 13, 12 }, { 106, 0xe20, 14, 13, 12 },
	{ 107, 0xe30, 2, 1, 0 }, { 108, 0xe30, 6, 5, 4 },
	{ 109, 0xe30, 10, 9, 8 }, { 110, 0xe10, 14, 13, 12 },
	{ 111, 0xe10, 10, 9, 8 }, { 112, 0xe10, 6, 5, 4 },
	{ 113, 0xe10, 2, 1, 0 }, { 114, 0xe20, 10, 9, 8 },
	{ 115, 0xe20, 2, 1, 0 }, { 116, 0xe20, 6, 5, 4 },
	{ 117, 0xe00, 14, 13, 12 }, { 118, 0xe00, 10, 9, 8 },
	{ 119, 0xe00, 6, 5, 4 }, { 120, 0xe00, 2, 1, 0 },
};

/* Input enable and Schmitt trigger bits, shared by a range of pins. */
struct mt8167_pin_range {
	uint8_t		start;
	uint8_t		end;
	uint16_t	reg;
	uint8_t		bit;
};

static const struct mt8167_pin_range mt8167_ies[] = {
	{ 0, 6, 0x900, 2 }, { 7, 10, 0x900, 3 }, { 11, 13, 0x900, 12 },
	{ 14, 17, 0x900, 13 }, { 18, 20, 0x910, 10 }, { 21, 23, 0x900, 13 },
	{ 24, 25, 0x900, 12 }, { 26, 30, 0x900, 0 }, { 31, 33, 0x900, 1 },
	{ 34, 39, 0x900, 2 }, { 40, 40, 0x910, 11 }, { 41, 43, 0x900, 10 },
	{ 44, 47, 0x900, 11 }, { 48, 51, 0x900, 14 }, { 52, 53, 0x910, 0 },
	{ 54, 54, 0x910, 2 }, { 55, 57, 0x910, 4 }, { 58, 59, 0x900, 15 },
	{ 60, 61, 0x910, 1 }, { 62, 65, 0x910, 5 }, { 66, 67, 0x910, 6 },
	{ 68, 68, 0x930, 2 }, { 69, 69, 0x930, 1 }, { 70, 70, 0x930, 6 },
	{ 71, 71, 0x930, 5 }, { 72, 72, 0x930, 4 }, { 73, 73, 0x930, 3 },
	{ 100, 103, 0x910, 7 }, { 104, 104, 0x920, 12 }, { 105, 105, 0x920, 11 },
	{ 106, 106, 0x930, 0 }, { 107, 107, 0x920, 15 }, { 108, 108, 0x920, 14 },
	{ 109, 109, 0x920, 13 }, { 110, 110, 0x920, 9 }, { 111, 111, 0x920, 8 },
	{ 112, 112, 0x920, 7 }, { 113, 113, 0x920, 6 }, { 114, 114, 0x920, 10 },
	{ 115, 115, 0x920, 1 }, { 116, 116, 0x920, 0 }, { 117, 117, 0x920, 5 },
	{ 118, 118, 0x920, 4 }, { 119, 119, 0x920, 3 }, { 120, 120, 0x920, 2 },
	{ 121, 124, 0x910, 9 },
};

static const struct mt8167_pin_range mt8167_smt[] = {
	{ 0, 6, 0xa00, 2 }, { 7, 10, 0xa00, 3 }, { 11, 13, 0xa00, 12 },
	{ 14, 17, 0xa00, 13 }, { 18, 20, 0xa10, 10 }, { 21, 23, 0xa00, 13 },
	{ 24, 25, 0xa00, 12 }, { 26, 30, 0xa00, 0 }, { 31, 33, 0xa00, 1 },
	{ 34, 39, 0xa00, 2 }, { 40, 40, 0xa10, 11 }, { 41, 43, 0xa00, 10 },
	{ 44, 47, 0xa00, 11 }, { 48, 51, 0xa00, 14 }, { 52, 53, 0xa10, 0 },
	{ 54, 54, 0xa10, 2 }, { 55, 57, 0xa10, 4 }, { 58, 59, 0xa00, 15 },
	{ 60, 61, 0xa10, 1 }, { 62, 65, 0xa10, 5 }, { 66, 67, 0xa10, 6 },
	{ 68, 68, 0xa30, 2 }, { 69, 69, 0xa30, 1 }, { 70, 70, 0xa30, 3 },
	{ 71, 71, 0xa30, 4 }, { 72, 72, 0xa30, 5 }, { 73, 73, 0xa30, 6 },
	{ 100, 103, 0xa10, 7 }, { 104, 104, 0xa20, 12 }, { 105, 105, 0xa20, 11 },
	{ 106, 106, 0xa30, 13 }, { 107, 107, 0xa20, 14 }, { 108, 108, 0xa20, 15 },
	{ 109, 109, 0xa30, 0 }, { 110, 110, 0xa20, 9 }, { 111, 111, 0xa20, 8 },
	{ 112, 112, 0xa20, 7 }, { 113, 113, 0xa20, 6 }, { 114, 114, 0xa20, 10 },
	{ 115, 115, 0xa20, 1 }, { 116, 116, 0xa20, 0 }, { 117, 117, 0xa20, 5 },
	{ 118, 118, 0xa20, 4 }, { 119, 119, 0xa20, 3 }, { 120, 120, 0xa20, 2 },
	{ 121, 124, 0xa10, 9 },
};

static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt8167-pinctrl",	1},
	{NULL,				0},
};

/* Write one bit through the SET or CLR companion of a register. */
static void
mt8167_pctl_setclr(struct mt8167_pinctrl_softc *sc, bus_size_t reg,
    uint32_t bit, bool set)
{

	WR4(sc, reg + (set ? PCTL_SET : PCTL_CLR), bit);
}

static void
mt8167_pctl_set_mode(struct mt8167_pinctrl_softc *sc, uint32_t pin,
    uint32_t mode)
{
	bus_size_t reg;
	uint32_t shift, val;

	reg = PCTL_MODE + ((pin / PCTL_MODE_PINS) << 4);
	shift = (pin % PCTL_MODE_PINS) * PCTL_MODE_BITS;

	PCTL_LOCK(sc);
	val = RD4(sc, reg);
	val &= ~(PCTL_MODE_MASK << shift);
	val |= (mode & PCTL_MODE_MASK) << shift;
	WR4(sc, reg, val);
	PCTL_UNLOCK(sc);
}

static void
mt8167_pctl_set_dir(struct mt8167_pinctrl_softc *sc, uint32_t pin,
    bool output)
{

	mt8167_pctl_setclr(sc, PCTL_DIR + PCTL_PORT(pin), PCTL_BIT(pin),
	    output);
}

static bool
mt8167_pctl_get_dir(struct mt8167_pinctrl_softc *sc, uint32_t pin)
{

	return ((RD4(sc, PCTL_DIR + PCTL_PORT(pin)) & PCTL_BIT(pin)) != 0);
}

static void
mt8167_pctl_set_out(struct mt8167_pinctrl_softc *sc, uint32_t pin, bool high)
{

	mt8167_pctl_setclr(sc, PCTL_DOUT + PCTL_PORT(pin), PCTL_BIT(pin),
	    high);
}

static int
mt8167_pctl_set_range(struct mt8167_pinctrl_softc *sc,
    const struct mt8167_pin_range *tbl, int n, uint32_t pin, bool enable)
{
	int i;

	for (i = 0; i < n; i++) {
		if (pin >= tbl[i].start && pin <= tbl[i].end) {
			mt8167_pctl_setclr(sc, tbl[i].reg, 1u << tbl[i].bit,
			    enable);
			return (0);
		}
	}
	return (EINVAL);
}

static int
mt8167_pctl_set_ies(struct mt8167_pinctrl_softc *sc, uint32_t pin,
    bool enable)
{

	return (mt8167_pctl_set_range(sc, mt8167_ies, nitems(mt8167_ies),
	    pin, enable));
}

static int
mt8167_pctl_set_smt(struct mt8167_pinctrl_softc *sc, uint32_t pin,
    bool enable)
{

	return (mt8167_pctl_set_range(sc, mt8167_smt, nitems(mt8167_smt),
	    pin, enable));
}

static int
mt8167_pctl_set_drive(struct mt8167_pinctrl_softc *sc, uint32_t pin,
    uint32_t ma)
{
	const struct mt8167_drv_class *class;
	const struct mt8167_pin_drv *drv;
	uint32_t mask, shift, val;
	int i;

	drv = NULL;
	for (i = 0; i < nitems(mt8167_pin_drv); i++) {
		if (mt8167_pin_drv[i].pin == pin) {
			drv = &mt8167_pin_drv[i];
			break;
		}
	}
	if (drv == NULL)
		return (EINVAL);

	class = &mt8167_drv_class[drv->class];
	if (ma < class->min || ma > class->max || (ma % class->step) != 0)
		return (EINVAL);

	shift = drv->bit + class->low_bit;
	mask = ((1u << (class->high_bit - class->low_bit + 1)) - 1) << shift;

	PCTL_LOCK(sc);
	val = RD4(sc, drv->reg);
	val &= ~mask;
	val |= ((ma / class->step - 1) << shift) & mask;
	WR4(sc, drv->reg, val);
	PCTL_UNLOCK(sc);

	return (0);
}

/*
 * Pull setup of the pins with a dedicated register.  Returns EINVAL when
 * the pin has none, or when r1r0 is not one of the MTK_PUPD_SET_R1R0_*
 * values of the device tree binding; the caller then falls back to the
 * common pull registers.
 */
static int
mt8167_pctl_set_pupd(struct mt8167_pinctrl_softc *sc, uint32_t pin, bool up,
    uint32_t r1r0)
{
	const struct mt8167_pin_pupd *pp;
	int i;

	pp = NULL;
	for (i = 0; i < nitems(mt8167_pin_pupd); i++) {
		if (mt8167_pin_pupd[i].pin == pin) {
			pp = &mt8167_pin_pupd[i];
			break;
		}
	}
	if (pp == NULL)
		return (EINVAL);

	/* The PUPD bit selects pull down when set. */
	mt8167_pctl_setclr(sc, pp->reg, 1u << pp->pupd_bit, !up);

	switch (r1r0) {
	case MTK_PUPD_SET_R1R0_00:
		mt8167_pctl_setclr(sc, pp->reg, 1u << pp->r0_bit, false);
		mt8167_pctl_setclr(sc, pp->reg, 1u << pp->r1_bit, false);
		break;
	case MTK_PUPD_SET_R1R0_01:
		mt8167_pctl_setclr(sc, pp->reg, 1u << pp->r0_bit, true);
		mt8167_pctl_setclr(sc, pp->reg, 1u << pp->r1_bit, false);
		break;
	case MTK_PUPD_SET_R1R0_10:
		mt8167_pctl_setclr(sc, pp->reg, 1u << pp->r0_bit, false);
		mt8167_pctl_setclr(sc, pp->reg, 1u << pp->r1_bit, true);
		break;
	case MTK_PUPD_SET_R1R0_11:
		mt8167_pctl_setclr(sc, pp->reg, 1u << pp->r0_bit, true);
		mt8167_pctl_setclr(sc, pp->reg, 1u << pp->r1_bit, true);
		break;
	default:
		return (EINVAL);
	}
	return (0);
}

static int
mt8167_pctl_set_pull(struct mt8167_pinctrl_softc *sc, uint32_t pin,
    bool enable, bool up, uint32_t arg)
{

	if (mt8167_pctl_set_pupd(sc, pin, up,
	    enable ? arg : MTK_PUPD_SET_R1R0_00) == 0)
		return (0);

	/* The common registers only know on and off. */
	if (arg != 0 && arg != 1)
		return (EINVAL);

	mt8167_pctl_setclr(sc, PCTL_PULLEN + PCTL_PORT(pin), PCTL_BIT(pin),
	    enable);
	mt8167_pctl_setclr(sc, PCTL_PULLSEL + PCTL_PORT(pin), PCTL_BIT(pin),
	    up);
	return (0);
}

/*
 * fdt_pinctrl
 */

/* Read a flag property with an optional cell, e.g. "bias-pull-up = <x>". */
static bool
mt8167_pctl_prop(phandle_t node, const char *name, uint32_t defval,
    uint32_t *val)
{

	if (!OF_hasprop(node, name))
		return (false);
	if (OF_getencprop(node, name, val, sizeof(*val)) <= 0)
		*val = defval;
	return (true);
}

static void
mt8167_pctl_warn(struct mt8167_pinctrl_softc *sc, uint32_t pin,
    const char *what, int rv)
{

	if (rv != 0)
		device_printf(sc->dev, "pin %u: cannot set %s\n", pin, what);
}

static void
mt8167_pctl_configure_pin(struct mt8167_pinctrl_softc *sc, phandle_t node,
    uint32_t pin)
{
	uint32_t arg;

	if (mt8167_pctl_prop(node, "bias-disable", 0, &arg))
		mt8167_pctl_warn(sc, pin, "bias-disable",
		    mt8167_pctl_set_pull(sc, pin, false, false, 0));
	if (mt8167_pctl_prop(node, "bias-pull-up", 1, &arg))
		mt8167_pctl_warn(sc, pin, "bias-pull-up",
		    mt8167_pctl_set_pull(sc, pin, true, true, arg));
	if (mt8167_pctl_prop(node, "bias-pull-down", 1, &arg))
		mt8167_pctl_warn(sc, pin, "bias-pull-down",
		    mt8167_pctl_set_pull(sc, pin, true, false, arg));

	if (mt8167_pctl_prop(node, "drive-strength", 0, &arg))
		mt8167_pctl_warn(sc, pin, "drive-strength",
		    mt8167_pctl_set_drive(sc, pin, arg));

	if (OF_hasprop(node, "input-enable") ||
	    OF_hasprop(node, "input-disable")) {
		mt8167_pctl_set_dir(sc, pin, false);
		mt8167_pctl_warn(sc, pin, "input enable",
		    mt8167_pctl_set_ies(sc, pin,
		    OF_hasprop(node, "input-enable")));
	}
	if (OF_hasprop(node, "input-schmitt-enable") ||
	    OF_hasprop(node, "input-schmitt-disable")) {
		mt8167_pctl_set_dir(sc, pin, false);
		mt8167_pctl_warn(sc, pin, "schmitt trigger",
		    mt8167_pctl_set_smt(sc, pin,
		    OF_hasprop(node, "input-schmitt-enable")));
	}

	if (OF_hasprop(node, "output-high") ||
	    OF_hasprop(node, "output-low")) {
		mt8167_pctl_set_out(sc, pin, OF_hasprop(node, "output-high"));
		mt8167_pctl_set_dir(sc, pin, true);
	}
}

/* One node with a "pinmux" list and the configuration shared by its pins. */
static void
mt8167_pctl_configure_node(struct mt8167_pinctrl_softc *sc, phandle_t node)
{
	pcell_t *pinmux;
	uint32_t pin;
	int i, npins;

	npins = OF_getencprop_alloc_multi(node, "pinmux", sizeof(*pinmux),
	    (void **)&pinmux);
	if (npins <= 0)
		return;

	for (i = 0; i < npins; i++) {
		pin = MTK_GET_PIN_NO(pinmux[i]);
		if (pin >= MT8167_NPINS) {
			device_printf(sc->dev, "invalid pin %u\n", pin);
			continue;
		}
		mt8167_pctl_set_mode(sc, pin, MTK_GET_PIN_FUNC(pinmux[i]));
		mt8167_pctl_configure_pin(sc, node, pin);
	}
	OF_prop_free(pinmux);
}

static int
mt8167_pinctrl_configure(device_t dev, phandle_t cfgxref)
{
	struct mt8167_pinctrl_softc *sc;
	phandle_t child, node;

	sc = device_get_softc(dev);
	node = OF_node_from_xref(cfgxref);

	mt8167_pctl_configure_node(sc, node);
	for (child = OF_child(node); child != 0; child = OF_peer(child))
		mt8167_pctl_configure_node(sc, child);

	return (0);
}

/*
 * GPIO
 */

static device_t
mt8167_gpio_get_bus(device_t dev)
{
	struct mt8167_pinctrl_softc *sc;

	sc = device_get_softc(dev);
	return (sc->busdev);
}

static int
mt8167_gpio_pin_max(device_t dev, int *maxpin)
{

	*maxpin = MT8167_NPINS - 1;
	return (0);
}

static int
mt8167_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{

	if (pin >= MT8167_NPINS)
		return (EINVAL);
	snprintf(name, GPIOMAXNAME, "GPIO%u", pin);
	return (0);
}

static int
mt8167_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{

	if (pin >= MT8167_NPINS)
		return (EINVAL);
	*caps = GPIO_PIN_INPUT | GPIO_PIN_OUTPUT | GPIO_PIN_PULLUP |
	    GPIO_PIN_PULLDOWN;
	return (0);
}

static int
mt8167_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct mt8167_pinctrl_softc *sc;

	if (pin >= MT8167_NPINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	*flags = mt8167_pctl_get_dir(sc, pin) ? GPIO_PIN_OUTPUT :
	    GPIO_PIN_INPUT;
	return (0);
}

static int
mt8167_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct mt8167_pinctrl_softc *sc;

	if (pin >= MT8167_NPINS)
		return (EINVAL);

	sc = device_get_softc(dev);

	/* The pull is left alone unless one is asked for. */
	if ((flags & GPIO_PIN_PULLUP) != 0)
		mt8167_pctl_set_pull(sc, pin, true, true, 1);
	else if ((flags & GPIO_PIN_PULLDOWN) != 0)
		mt8167_pctl_set_pull(sc, pin, true, false, 1);

	if ((flags & GPIO_PIN_OUTPUT) != 0) {
		mt8167_pctl_set_dir(sc, pin, true);
	} else if ((flags & GPIO_PIN_INPUT) != 0) {
		mt8167_pctl_set_dir(sc, pin, false);
		mt8167_pctl_set_ies(sc, pin, true);
	}
	mt8167_pctl_set_mode(sc, pin, PCTL_MODE_GPIO);

	return (0);
}

static int
mt8167_gpio_pin_get(device_t dev, uint32_t pin, uint32_t *val)
{
	struct mt8167_pinctrl_softc *sc;

	if (pin >= MT8167_NPINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	*val = (RD4(sc, PCTL_DIN + PCTL_PORT(pin)) & PCTL_BIT(pin)) != 0;
	return (0);
}

static int
mt8167_gpio_pin_set(device_t dev, uint32_t pin, uint32_t val)
{
	struct mt8167_pinctrl_softc *sc;

	if (pin >= MT8167_NPINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	mt8167_pctl_set_out(sc, pin, val != 0);
	return (0);
}

static int
mt8167_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct mt8167_pinctrl_softc *sc;
	bool high;

	if (pin >= MT8167_NPINS)
		return (EINVAL);

	sc = device_get_softc(dev);
	PCTL_LOCK(sc);
	high = (RD4(sc, PCTL_DOUT + PCTL_PORT(pin)) & PCTL_BIT(pin)) != 0;
	mt8167_pctl_set_out(sc, pin, !high);
	PCTL_UNLOCK(sc);
	return (0);
}

static phandle_t
mt8167_gpio_get_node(device_t bus, device_t dev)
{

	return (ofw_bus_get_node(bus));
}

/*
 * Device
 */

static int
mt8167_pinctrl_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	device_set_desc(dev, "MediaTek MT8167 pinctrl/GPIO controller");
	return (BUS_PROBE_DEFAULT);
}

static int
mt8167_pinctrl_attach(device_t dev)
{
	struct mt8167_pinctrl_softc *sc;
	phandle_t node, regnode;
	pcell_t xref;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	/*
	 * The pin registers belong to the syscfg_pctl syscon node.  That one
	 * is attached by the generic syscon driver long after this driver,
	 * which has to run early enough to set the pins up for everybody
	 * else, so map the registers of the node directly.
	 */
	if (OF_getencprop(node, "mediatek,pctl-regmap", &xref,
	    sizeof(xref)) <= 0) {
		device_printf(dev, "no mediatek,pctl-regmap property\n");
		return (ENXIO);
	}
	regnode = OF_node_from_xref(xref);
	if (OF_decode_addr(regnode, 0, &sc->bst, &sc->bsh, &sc->size) != 0) {
		device_printf(dev, "cannot map the pin registers\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_SPIN);

	fdt_pinctrl_register(dev, NULL);
	fdt_pinctrl_configure_tree(dev);

	sc->busdev = gpiobus_add_bus(dev);
	if (sc->busdev == NULL) {
		device_printf(dev, "cannot attach gpiobus\n");
		mtx_destroy(&sc->mtx);
		return (ENXIO);
	}

	bus_attach_children(dev);
	return (0);
}

static int
mt8167_pinctrl_detach(device_t dev)
{

	/* Other drivers hold references to the pin configurations. */
	return (EBUSY);
}

static device_method_t mt8167_pinctrl_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mt8167_pinctrl_probe),
	DEVMETHOD(device_attach,	mt8167_pinctrl_attach),
	DEVMETHOD(device_detach,	mt8167_pinctrl_detach),

	/* GPIO interface */
	DEVMETHOD(gpio_get_bus,		mt8167_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		mt8167_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	mt8167_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getcaps,	mt8167_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_getflags,	mt8167_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	mt8167_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		mt8167_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		mt8167_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	mt8167_gpio_pin_toggle),

	/* fdt_pinctrl interface */
	DEVMETHOD(fdt_pinctrl_configure, mt8167_pinctrl_configure),

	/* ofw_bus interface */
	DEVMETHOD(ofw_bus_get_node,	mt8167_gpio_get_node),

	DEVMETHOD_END
};

extern driver_t ofw_gpiobus_driver;

static DEFINE_CLASS_0(gpio, mt8167_pinctrl_driver, mt8167_pinctrl_methods,
    sizeof(struct mt8167_pinctrl_softc));
EARLY_DRIVER_MODULE(mt8167_pinctrl, simplebus, mt8167_pinctrl_driver, NULL,
    NULL, BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
EARLY_DRIVER_MODULE(ofw_gpiobus, mt8167_pinctrl, ofw_gpiobus_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
MODULE_VERSION(mt8167_pinctrl, 1);
