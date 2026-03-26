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

#define MT_UART_RATE_FIX     0x0d /* Rate fix register */
#define MT_UART_HIGHS	     0x09 /* Highspeed mode register */
#define MT_UART_SAMPLE_COUNT 0x0a /* Sample count register */
#define MT_UART_SAMPLE_POINT 0x0b /* Sample point register */
#define MT_UART_FRACDIV_L    0x15 /* Fractional divisor LSB */
#define MT_UART_FRACDIV_M    0x16 /* Fractional divisor MSB */

/*
 * Fractional divisor lookup tables.
 *
 * Indexed by (fraction / 10), where:
 *   fraction = ((rclk * 100) / baud / quot) % 100
 *
 * These come directly from the Linux mtk8250 driver and are used to
 * fine-tune the baud rate accuracy in highspeed mode 3.
 */
static const uint8_t mt_fraction_L_mapping[] = { 0x00, 0x01, 0x05, 0x15, 0x55,
	0x57, 0x57, 0x77, 0x7F, 0xFF, 0xFF };

static const uint8_t mt_fraction_M_mapping[] = { 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
	3 };

/*
 * TODO: Override low-level ops (.init) to support MTK highspeed
 * mode 3 for console path. Currently the console uses standard
 * ns8250 16x divisor, which works for baud rates that divide
 * evenly from rclk (e.g. 25MHz/16/1 = 1562500).
 *
 * For arbitrary baud rates (115200, 921600, etc.), MTK highspeed
 * mode 3 with sample_count/sample_point and fractional divisor
 * is needed for accurate clocking. See Linux 8250_mtk.c.
 *
 */

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
mt_bus_param(struct uart_softc *sc, int baudrate, int databits, int stopbits,
    int parity)
{
	struct uart_bas *bas;
	uint8_t lcr;

	bas = &sc->sc_bas;
	lcr = 0;

	/* Data bits */
	switch (databits) {
	case 5:
		lcr |= CFCR_5BITS;
		break;
	case 6:
		lcr |= CFCR_6BITS;
		break;
	case 7:
		lcr |= CFCR_7BITS;
		break;
	case 8:
	default:
		lcr |= CFCR_8BITS;
		break;
	}

	/* Stop bits */
	if (stopbits == 2)
		lcr |= CFCR_STOPB;

	/* Parity */
	switch (parity) {
	case UART_PARITY_EVEN:
		lcr |= CFCR_PENAB | CFCR_PEVEN;
		break;
	case UART_PARITY_ODD:
		lcr |= CFCR_PENAB;
		break;
	case UART_PARITY_MARK:
		lcr |= CFCR_PENAB | CFCR_PONE;
		break;
	case UART_PARITY_SPACE:
		lcr |= CFCR_PENAB | CFCR_PZERO;
		break;
	case UART_PARITY_NONE:
	default:
		break;
	}

	uart_lock(sc->sc_hwmtx);

	/* Set line control register */
	uart_setreg(bas, REG_LCR, lcr);
	uart_barrier(bas);

	/* Set baud rate with MTK-specific logic */
	if (baudrate > 0 && bas->rclk > 0) {
		uint32_t quot, fraction, tmp;

		if (baudrate < 115200) {
			/* Standard mode: 16x oversampling */
			uart_setreg(bas, MT_UART_HIGHS, 0x0);
			uart_barrier(bas);

			quot = (bas->rclk + 8 * baudrate) / (16 * baudrate);
			if (quot == 0)
				quot = 1;
		} else {
			/* High-speed mode 3 */
			uart_setreg(bas, MT_UART_HIGHS, 0x3);
			uart_barrier(bas);

			quot = (bas->rclk + 256 * baudrate - 1) /
			    (256 * baudrate);
			if (quot == 0)
				quot = 1;
		}

		/* Program divisor latch */
		uart_setreg(bas, REG_LCR, lcr | CFCR_DLAB);
		uart_barrier(bas);

		uart_setreg(bas, REG_DATA, quot & 0xFF);
		uart_setreg(bas, REG_IER, (quot >> 8) & 0xFF);
		uart_barrier(bas);

		uart_setreg(bas, REG_LCR, lcr);
		uart_barrier(bas);

		/* Program sample count/point and fractional divisor */
		if (baudrate >= 115200) {
			tmp = (bas->rclk / (baudrate * quot)) - 1;
			uart_setreg(bas, MT_UART_SAMPLE_COUNT, tmp);
			uart_setreg(bas, MT_UART_SAMPLE_POINT, (tmp >> 1) - 1);
			uart_barrier(bas);

			fraction = ((bas->rclk * 100) / baudrate / quot) % 100;
			fraction = (fraction + 5) / 10;
			if (fraction > 10)
				fraction = 10;

			uart_setreg(bas, MT_UART_FRACDIV_L,
			    mt_fraction_L_mapping[fraction]);
			uart_setreg(bas, MT_UART_FRACDIV_M,
			    mt_fraction_M_mapping[fraction]);
		} else {
			uart_setreg(bas, MT_UART_SAMPLE_COUNT, 0x00);
			uart_setreg(bas, MT_UART_SAMPLE_POINT, 0xFF);
			uart_setreg(bas, MT_UART_FRACDIV_L, 0x00);
			uart_setreg(bas, MT_UART_FRACDIV_M, 0x00);
		}
		uart_barrier(bas);
	}

	uart_unlock(sc->sc_hwmtx);

	return (0);
}

static kobj_method_t mdtk_methods[] = {
	KOBJMETHOD(uart_probe,		ns8250_bus_probe),
    KOBJMETHOD(uart_attach,		mdtk_uart_attach),
	KOBJMETHOD(uart_detach,		ns8250_bus_detach),
	KOBJMETHOD(uart_flush,		ns8250_bus_flush),
	KOBJMETHOD(uart_getsig,		ns8250_bus_getsig),
	KOBJMETHOD(uart_ioctl,		ns8250_bus_ioctl),
	KOBJMETHOD(uart_ipend,		ns8250_bus_ipend),
	KOBJMETHOD(uart_param,		mt_bus_param),
	KOBJMETHOD(uart_receive,	ns8250_bus_receive),
	KOBJMETHOD(uart_setsig,		ns8250_bus_setsig),
	KOBJMETHOD(uart_transmit,	ns8250_bus_transmit),
	KOBJMETHOD(uart_txbusy,		ns8250_bus_txbusy),
    KOBJMETHOD(uart_grab,		ns8250_bus_grab),
    KOBJMETHOD(uart_ungrab,		ns8250_bus_ungrab),
	KOBJMETHOD_END
};

static struct uart_class mdtk_uart_class = {
    "mediatek class",
    mdtk_methods,
    sizeof(struct mdtk_softc),
	.uc_ops = &uart_ns8250_ops,
	.uc_range = 8,
	.uc_rclk = 0,  
    .uc_rshift = 2,
    .uc_riowidth = 4,
};

/* Compatible devices. */
static struct ofw_compat_data compat_data[] = {
    {"mediatek,mt7622-uart",(uintptr_t)&mdtk_uart_class},
    {"mediatek,mt6577-uart",(uintptr_t)&mdtk_uart_class},
	{NULL,			 (uintptr_t)NULL},
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
mdtk_uart_probe(device_t dev)
{
    struct mdtk_softc *sc;
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
	rv = clk_get_by_ofw_index(dev, 0, 0, &sc->clk);
	if (rv != 0) {
		device_printf(dev, "Cannot get UART clock: %d\n", rv);
		return (ENXIO);
	}
	rv = clk_enable(sc->clk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable UART clock: %d\n", rv);
		return (ENXIO);
	}
	rv = clk_get_freq(sc->clk, &freq);
	if (rv != 0) {
		device_printf(dev, "Cannot enable UART clock: %d\n", rv);
		return (ENXIO);
	}

	return (uart_bus_probe(dev, shift, 0, (int)freq, 0, 0, 0));
}

static int
mdtk_uart_detach(device_t dev)
{
    struct mdtk_softc *sc;

	sc = device_get_softc(dev);
	if (sc->clk != NULL) {
		clk_release(sc->clk);
	}

	return (uart_bus_detach(dev));
}

static device_method_t mdtk_uart_bus_methods[] = {
	/* Device interface */
    DEVMETHOD(device_probe,		mdtk_uart_probe),
	DEVMETHOD(device_attach,	uart_bus_attach),
    DEVMETHOD(device_detach,	mdtk_uart_detach),
	DEVMETHOD_END
};

static driver_t mdtk_uart_driver = {
	uart_driver_name,
    mdtk_uart_bus_methods,
    sizeof(struct mdtk_softc),
};

DRIVER_MODULE(mdtk_uart, simplebus,  mdtk_uart_driver, 0, 0);
