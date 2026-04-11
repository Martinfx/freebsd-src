/*-
 * Copyright (c) 2025 Martin Filla, Michal Meloun <mmel@FreeBSD.org>
 * All rights reserved.
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

#include <sys/cdefs.h>
/*
 * UART driver for mediatek SoCs.
 */
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/sysctl.h>

#include <machine/bus.h>

#include <dev/clk/clk.h>
#include <dev/ic/ns16550.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/uart/uart.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_cpu_fdt.h>
#include <dev/uart/uart_dev_ns8250.h>

#include "uart_if.h"

#define	MT_UART_HIGHS		0x09
#define	MT_UART_SAMPLE_COUNT	0x0a
#define	MT_UART_SAMPLE_POINT	0x0b
#define	MT_UART_RATE_FIX	0x0d
#define	MT_UART_FRACDIV_L	0x15
#define	MT_UART_FRACDIV_M	0x16

/* Fractional divisor lookup tables (from Linux 8250_mtk.c) */
static const uint8_t mt_fraction_L_mapping[] = {
	0x00, 0x01, 0x05, 0x15, 0x55, 0x57, 0x57, 0x77, 0x7F, 0xFF, 0xFF
};
static const uint8_t mt_fraction_M_mapping[] = {
	0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 3
};

/*
 * High-level UART interface.
 */
struct mt_softc {
	struct ns8250_softc ns8250_base;
	clk_t baud_clk;
	clk_t bus_clk;
};

static int
mt_uart_attach(struct uart_softc *sc)
{
	int rv;
	struct ns8250_softc *ns8250 = (struct ns8250_softc *)sc;
	struct uart_bas *bas = &sc->sc_bas;

	rv = ns8250_bus_attach(sc);
	if (rv != 0)
		return (rv);

	ns8250->ier_mask = 0xc0;
	ns8250->ier = uart_getreg(bas, REG_IER) & ns8250->ier_mask;
	ns8250->ier |= ns8250->ier_rxbits;
	uart_setreg(bas, REG_IER, ns8250->ier);
	uart_barrier(bas);

	// Some MediaTek SoCs need this to enable correct clock dividing.
	uart_setreg(bas, MT_UART_RATE_FIX, 0x00);
	uart_barrier(bas);

	return (0);
}

static int
mt_calc_divisor(struct uart_softc *sc, int baudrate)
{
	struct uart_bas *bas = &sc->sc_bas;
	uint32_t divisor, sample_actual, fraction;

	/* Low baud rates: clear MTK registers, fall back to standard 16x */
	if (baudrate < 115200) {
		uart_setreg(bas, MT_UART_HIGHS, 0x0);
		uart_setreg(bas, MT_UART_SAMPLE_COUNT, 0x00);
		uart_setreg(bas, MT_UART_SAMPLE_POINT, 0xFF);
		uart_setreg(bas, MT_UART_FRACDIV_L, 0x00);
		uart_setreg(bas, MT_UART_FRACDIV_M, 0x00);
		uart_barrier(bas);
		return (0);
	}

	/* Highspeed mode 3 */
	divisor = (bas->rclk + 256 * baudrate - 1) / (256 * baudrate);
	if (divisor == 0)
		divisor = 1;
	if (divisor >= 65536)
		return (0);

	sample_actual = bas->rclk / (baudrate * divisor);
	if (sample_actual == 0)
		return (0);

	/* Switch to mode 3 BEFORE DLL/DLM is written by caller */
	uart_setreg(bas, MT_UART_HIGHS, 0x3);
	uart_barrier(bas);

	/* sample_count register = actual - 1 (counter 0..reg) */
	uart_setreg(bas, MT_UART_SAMPLE_COUNT, sample_actual - 1);
	/* sample_point = actual / 2 per datasheet page 159 */
	uart_setreg(bas, MT_UART_SAMPLE_POINT, sample_actual / 2);
	uart_barrier(bas);

	fraction = ((bas->rclk * 100) / baudrate / divisor) % 100;
	fraction = (fraction + 5) / 10;
	if (fraction > 10)
		fraction = 10;
	uart_setreg(bas, MT_UART_FRACDIV_L, mt_fraction_L_mapping[fraction]);
	uart_setreg(bas, MT_UART_FRACDIV_M, mt_fraction_M_mapping[fraction]);
	uart_barrier(bas);

	return (divisor);
}

static kobj_method_t mt_methods[] = {
	KOBJMETHOD(uart_probe,		ns8250_bus_probe),
    KOBJMETHOD(uart_attach,		mt_uart_attach),
	KOBJMETHOD(uart_detach,		ns8250_bus_detach),
	KOBJMETHOD(uart_flush,		ns8250_bus_flush),
	KOBJMETHOD(uart_getsig,		ns8250_bus_getsig),
	KOBJMETHOD(uart_ioctl,		ns8250_bus_ioctl),
	KOBJMETHOD(uart_ipend,		ns8250_bus_ipend),
	KOBJMETHOD(uart_param, 		ns8250_bus_param),
	KOBJMETHOD(uart_receive,	ns8250_bus_receive),
	KOBJMETHOD(uart_setsig,		ns8250_bus_setsig),
	KOBJMETHOD(uart_transmit,	ns8250_bus_transmit),
	KOBJMETHOD(uart_txbusy,		ns8250_bus_txbusy),
    KOBJMETHOD(uart_grab,		ns8250_bus_grab),
    KOBJMETHOD(uart_ungrab,		ns8250_bus_ungrab),
	KOBJMETHOD_END
};

static struct uart_class mt_uart_class = {
    "mediatek class",
	mt_methods,
    sizeof(struct mt_softc),
	.uc_ops = &uart_ns8250_ops,
	.uc_range = 8,
	.uc_rclk = 0,  
    .uc_rshift = 2,
    .uc_riowidth = 4,
	.uc_divisor = mt_calc_divisor,
};

/* Compatible devices. */
static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt6577-uart",(uintptr_t)&mt_uart_class},
	{NULL, 0}
};

UART_FDT_CLASS(compat_data);

/*
 * UART Driver interface.
 */
static int
uart_fdt_get_shift1(phandle_t node)
{
	pcell_t shift;

	if ((OF_getencprop(node, "reg-shift", &shift, sizeof(shift))) <= 0)
		shift = 2;
	return ((int)shift);
}

static int
mt_uart_probe(device_t dev)
{
    	struct mt_softc *sc;
	phandle_t node;
	uint64_t freq;
	int shift;
	int rv;
	const struct ofw_compat_data *cd;

	sc = device_get_softc(dev);
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	cd = ofw_bus_search_compatible(dev, compat_data);
	if (cd->ocd_data == 0)
		return (ENXIO);
	sc->ns8250_base.base.sc_class = (struct uart_class *)cd->ocd_data;

	node = ofw_bus_get_node(dev);
	shift = uart_fdt_get_shift1(node);

	rv = clk_get_by_ofw_name(dev, 0, "baud", &sc->baud_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot get 'baud' clock\n");
		return (ENXIO);
	}
	rv = clk_enable(sc->baud_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable baud UART clock: %d\n", rv);
		return (ENXIO);
	}

	rv = clk_get_by_ofw_name(dev, 0, "bus", &sc->bus_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot get 'bus' clock\n");
		return (ENXIO);
	}
	rv = clk_enable(sc->bus_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable bus UART clock: %d\n", rv);
		return (ENXIO);
	}

	rv = clk_get_freq(sc->baud_clk, &freq);
	if (rv != 0) {
		device_printf(dev, "Cannot enable UART clock: %d\n", rv);
		return (ENXIO);
	}

	return (uart_bus_probe(dev, shift, 0, (int)freq, 0, 0, 0));
}

static int
mt_uart_detach(device_t dev)
{
    	struct mt_softc *sc;

	sc = device_get_softc(dev);
	if (sc->baud_clk != NULL) {
		clk_release(sc->baud_clk);
	}

	if (sc->bus_clk != NULL) {
		clk_release(sc->bus_clk);
	}

	return (uart_bus_detach(dev));
}

static device_method_t mt_uart_bus_methods[] = {
	/* Device interface */
    	DEVMETHOD(device_probe,		mt_uart_probe),
	DEVMETHOD(device_attach,	uart_bus_attach),
    	DEVMETHOD(device_detach,	mt_uart_detach),
	DEVMETHOD_END
};

static driver_t mt_uart_driver = {
	uart_driver_name,
	mt_uart_bus_methods,
    	sizeof(struct mt_softc),
};

DRIVER_MODULE(mdtk_uart, simplebus,  mt_uart_driver, 0, 0);
