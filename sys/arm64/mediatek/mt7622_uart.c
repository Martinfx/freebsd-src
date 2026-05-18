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
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_cpu_fdt.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_dev_ns8250.h>
#include <dev/ic/ns16550.h>

#include "uart_if.h"

#define	MTK_UART_HIGHS		0x09	/* HIGHSPEED mode select (0..3) */
#define	MTK_UART_SAMPLE_COUNT	0x0a	/* sample count, jen pro highs==3 */
#define	MTK_UART_SAMPLE_POINT	0x0b	/* sample point, jen pro highs==3 */
#define MTK_UART_RATE_FIX	0x0d
#define UART_DIV_MAX		0xFFFF	/* Max divisor value */
#define MTK_UART_FRACDIV_L	0x15	/* Fractional divider LSB address */
#define MTK_UART_FRACDIV_M	0x16	/* Fractional divider MSB address */
/*
 * High-level UART interface.
 */
struct mt_softc {
	struct ns8250_softc 	ns8250_base;
	clk_t			baud, bus;
};

static uint32_t
get_compute_baudrate2(struct uart_bas *bas, struct uart_softc *sc)
{
	uint32_t baudrate, data_bits, stop_bits, parity_bit;
	uint32_t frame_bits, extra_clocks = 0;
	uint16_t divisor;
	uint8_t lcr, highs, sample_count;
	uint8_t fracdiv_l, fracdiv_m;

	highs = uart_getreg(bas, MTK_UART_HIGHS) & 0x3;

	lcr = uart_getreg(bas, REG_LCR);
	uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
	uart_barrier(bas);

	divisor = uart_getreg(bas, REG_DLH) << 8 | uart_getreg(bas, REG_DLL);

	uart_setreg(bas, REG_LCR, lcr);
	uart_barrier(bas);

	sample_count = uart_getreg(bas, MTK_UART_SAMPLE_COUNT);
	fracdiv_l    = uart_getreg(bas, MTK_UART_FRACDIV_L);
	fracdiv_m    = uart_getreg(bas, MTK_UART_FRACDIV_M) & 0x3;

	data_bits  = (lcr & 0x3) + 5;
	stop_bits  = (lcr & LCR_STOPB) ? 2 : 1;
	parity_bit = (lcr & LCR_PENAB) ? 1 : 0;

	frame_bits = 1 + data_bits + parity_bit + stop_bits;
	
	switch (highs) {
	case 0:
		baudrate = bas->rclk / 16 / divisor;
		break;
	case 1:
		baudrate = bas->rclk / 8 / divisor;
		break;
	case 2:
		baudrate = bas->rclk / 4 / divisor;
		break;
	case 3:

		for (int i = 0; i < data_bits; i++) {
			extra_clocks += (fracdiv_l >> i) & 0x1;
		}

		if (parity_bit) {
			extra_clocks += (fracdiv_m & 0x1);
		}

		if (stop_bits > 0) {
			extra_clocks += ((fracdiv_m >> 1) & 0x1);
		}

		uint32_t total_samples_per_char = (frame_bits * sample_count) + extra_clocks;

		if (divisor == 0 || total_samples_per_char == 0) {
			baudrate = 0;
		} else {
			baudrate = ((uint32_t)bas->rclk * frame_bits) / (divisor * total_samples_per_char);
		}
		break;
	default:
		device_printf(sc->sc_dev, "Bad highs=%d\n", highs);
		baudrate = 0;
		break;
	}

	device_printf(sc->sc_dev,
	    "rclk=%u highs=%u sample_count=%u divisor=%u "
	    "fracdiv_l=0x%02x fracdiv_m=0x%02x baudrate=%u\n",
	    bas->rclk, highs, sample_count, divisor,
	    fracdiv_l, fracdiv_m, baudrate);

	return baudrate;
}

/*
 * UART class interface.
 */
static int
mdtk_uart_attach(struct uart_softc *sc)
{
	int rv;
	struct ns8250_softc *ns8250 = (struct ns8250_softc*)sc;
	struct uart_bas *bas = &sc->sc_bas;

	rv = ns8250_bus_attach(sc);
	if (rv != 0)
		return (rv);

	uart_setreg(bas, MTK_UART_RATE_FIX, 0x0);

	ns8250->ier = uart_getreg(bas, REG_IER) & ns8250->ier_mask;
	ns8250->ier |= ns8250->ier_rxbits;
	uart_setreg(bas, REG_IER, ns8250->ier);
	uart_barrier(bas);

	//device_printf(sc->sc_dev, "UART baudrate: %d\n", get_compute_baudrate(bas, sc));
	device_printf(sc->sc_dev, "UART baudrate2: %d\n", get_compute_baudrate2(bas, sc));;
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
	KOBJMETHOD(uart_param,		ns8250_bus_param),
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
    mdtk_methods,
	sizeof(struct mt_softc),
	.uc_ops = &uart_ns8250_ops,
	.uc_range = 0x100,
	.uc_rclk = 0,  
    	.uc_rshift = 2,
    	.uc_riowidth = 4,
};

/* Compatible devices. */
static struct ofw_compat_data compat_data[] = {
    {"mediatek,mt6577-uart",(uintptr_t)&mt_uart_class},
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
	rv = clk_get_by_ofw_name(dev, 0, "baud", &sc->baud);
	if (rv != 0) {
		device_printf(dev, "Cannot get baud clock: %d\n", rv);
		return (ENXIO);
	}
	rv = clk_enable(sc->baud);
	if (rv != 0) {
		device_printf(dev, "Cannot enable UART clock: %d\n", rv);
		return (ENXIO);
	}

	rv = clk_get_by_ofw_name(dev, 0, "bus", &sc->bus);
	if (rv != 0) {
		device_printf(dev, "Cannot get UART clock: %d\n", rv);
		return (ENXIO);
	}
	rv = clk_enable(sc->bus);
	if (rv != 0) {
		device_printf(dev, "Cannot enable bus clock: %d\n", rv);
		return (ENXIO);
	}

	rv = clk_get_freq(sc->baud, &freq);
	if (rv != 0) {
		device_printf(dev, "Cannot get baud clock frequency: %d\n", rv);
		return (ENXIO);
	}

	return (uart_bus_probe(dev, shift, 0, (int)freq, 0, 0, 0));
}

static int
mt_uart_detach(device_t dev)
{
	struct mt_softc *sc;

	sc = device_get_softc(dev);
	if (sc->baud != NULL) {
		clk_release(sc->baud);
	}

	if (sc->bus != NULL) {
		clk_release(sc->bus);
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

DRIVER_MODULE(mt_uart, simplebus,  mt_uart_driver, 0, 0);
