/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 martinfx
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
 * MediaTek "mt6577" UART, as found in the MT8167/MT8516 family (Google
 * Coral Dev Board Mini, Pumpkin boards, ...).
 *
 * The block is a 16550 with a 4-byte register stride plus MediaTek
 * extensions for high-speed baud rate generation (HIGHSPEED, SAMPLE_COUNT,
 * SAMPLE_POINT and the fractional divider).  We always run the block with
 * HIGHSPEED == 0, where the baud rate is derived from the classic 16x
 * oversampling divisor, so the generic ns8250 code can drive it unchanged.
 * Rates above 115200 would need the high-speed modes and are not supported
 * yet.
 *
 * The baud clock is the 26 MHz crystal (clk26m) on every known SoC.  Until
 * FreeBSD has a MediaTek clock driver the clock controller nodes stay
 * unattached, so the frequency comes from the class default or from the
 * FreeBSD-only "clock-frequency" property in the board DTS.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_bus.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_cpu_fdt.h>
#include <dev/uart/uart_dev_ns8250.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>

#include "uart_if.h"

/*
 * MediaTek-specific registers.  Expressed as register indices; the byte
 * offset is index << regshft (regshft is 2 on this hardware).
 */
#define	MTK_REG_HIGHSPEED	(0x24 >> 2)
#define	 MTK_HIGHSPEED_16X	0	/* 16x oversampling, plain divisor */
#define	MTK_REG_SAMPLE_COUNT	(0x28 >> 2)
#define	MTK_REG_SAMPLE_POINT	(0x2c >> 2)
#define	MTK_REG_FRACDIV_L	(0x54 >> 2)
#define	MTK_REG_FRACDIV_M	(0x58 >> 2)

#define	MTK_UART_DEFAULT_RCLK	26000000	/* clk26m */
#define	MTK_UART_REG_SHIFT	2
#define	MTK_UART_REG_IO_WIDTH	4
#define	MTK_UART_NREGS		32		/* covers FRACDIV_M */

struct mtk_uart_softc {
	struct ns8250_softc	ns8250;
	clk_t			baudclk;
	clk_t			busclk;
};

/*
 * Force the classic 16550 baud rate generator.  The firmware may have left
 * the block in one of the high-speed modes, in which case the DLL/DLM
 * divisor programmed by the ns8250 code would give the wrong rate.
 */
static void
mtk_uart_set_16x_mode(struct uart_bas *bas)
{

	uart_setreg(bas, MTK_REG_HIGHSPEED, MTK_HIGHSPEED_16X);
	uart_barrier(bas);
}

/*
 * Low-level (console) operations.  These wrap the ns8250 ones so the
 * MediaTek speed mode is reset before any divisor is programmed.
 */
static int
mtk_uart_probe(struct uart_bas *bas)
{

	return (uart_ns8250_ops.probe(bas));
}

static void
mtk_uart_init(struct uart_bas *bas, int baudrate, int databits, int stopbits,
    int parity)
{

	mtk_uart_set_16x_mode(bas);
	uart_ns8250_ops.init(bas, baudrate, databits, stopbits, parity);
}

static void
mtk_uart_term(struct uart_bas *bas)
{

	uart_ns8250_ops.term(bas);
}

static void
mtk_uart_putc(struct uart_bas *bas, int c)
{

	uart_ns8250_ops.putc(bas, c);
}

static int
mtk_uart_rxready(struct uart_bas *bas)
{

	return (uart_ns8250_ops.rxready(bas));
}

static int
mtk_uart_getc(struct uart_bas *bas, struct mtx *hwmtx)
{

	return (uart_ns8250_ops.getc(bas, hwmtx));
}

static struct uart_ops mtk_uart_ops = {
	.probe = mtk_uart_probe,
	.init = mtk_uart_init,
	.term = mtk_uart_term,
	.putc = mtk_uart_putc,
	.rxready = mtk_uart_rxready,
	.getc = mtk_uart_getc,
};

/*
 * High-level (tty) operations.
 */
static int
mtk_uart_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{

	mtk_uart_set_16x_mode(&sc->sc_bas);
	return (ns8250_bus_param(sc, baudrate, databits, stopbits, parity));
}

static kobj_method_t mtk_uart_methods[] = {
	KOBJMETHOD(uart_probe,		ns8250_bus_probe),
	KOBJMETHOD(uart_attach,		ns8250_bus_attach),
	KOBJMETHOD(uart_detach,		ns8250_bus_detach),
	KOBJMETHOD(uart_flush,		ns8250_bus_flush),
	KOBJMETHOD(uart_getsig,		ns8250_bus_getsig),
	KOBJMETHOD(uart_ioctl,		ns8250_bus_ioctl),
	KOBJMETHOD(uart_ipend,		ns8250_bus_ipend),
	KOBJMETHOD(uart_param,		mtk_uart_bus_param),
	KOBJMETHOD(uart_receive,	ns8250_bus_receive),
	KOBJMETHOD(uart_setsig,		ns8250_bus_setsig),
	KOBJMETHOD(uart_transmit,	ns8250_bus_transmit),
	KOBJMETHOD(uart_txbusy,		ns8250_bus_txbusy),
	KOBJMETHOD(uart_grab,		ns8250_bus_grab),
	KOBJMETHOD(uart_ungrab,		ns8250_bus_ungrab),
	KOBJMETHOD_END
};

static struct uart_class uart_mtk_class = {
	"mtk",
	mtk_uart_methods,
	sizeof(struct mtk_uart_softc),
	.uc_ops = &mtk_uart_ops,
	.uc_range = MTK_UART_NREGS,
	.uc_rclk = MTK_UART_DEFAULT_RCLK,
	.uc_rshift = MTK_UART_REG_SHIFT,
	.uc_riowidth = MTK_UART_REG_IO_WIDTH,
};

static struct ofw_compat_data compat_data[] = {
	{ "mediatek,mt6577-uart",	(uintptr_t)&uart_mtk_class },
	{ NULL,				(uintptr_t)NULL }
};
UART_FDT_CLASS(compat_data);

static int
mtk_uart_fdt_probe(device_t dev)
{
	struct mtk_uart_softc *sc;
	phandle_t node;
	uint64_t freq;
	uint32_t shift, iowidth, clock;
	clk_t baudclk, busclk;
	int error;

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);
	if (ofw_bus_search_compatible(dev, compat_data)->ocd_data == 0)
		return (ENXIO);

	sc = device_get_softc(dev);
	sc->ns8250.base.sc_class = &uart_mtk_class;

	node = ofw_bus_get_node(dev);
	if (OF_getencprop(node, "reg-shift", &shift, sizeof(shift)) <= 0)
		shift = MTK_UART_REG_SHIFT;
	if (OF_getencprop(node, "reg-io-width", &iowidth, sizeof(iowidth)) <= 0)
		iowidth = MTK_UART_REG_IO_WIDTH;
	if (OF_getencprop(node, "clock-frequency", &clock, sizeof(clock)) <= 0)
		clock = 0;

	/*
	 * The clocks are optional: without a MediaTek clock driver the
	 * providers never attach and we rely on the firmware having left the
	 * UART clocked.
	 */
	baudclk = NULL;
	busclk = NULL;
	if (clk_get_by_ofw_name(dev, 0, "baud", &baudclk) == 0) {
		error = clk_enable(baudclk);
		if (error != 0) {
			device_printf(dev, "cannot enable baud clock\n");
			return (error);
		}
		if (clock == 0 && clk_get_freq(baudclk, &freq) == 0)
			clock = (uint32_t)freq;
	}
	if (clk_get_by_ofw_name(dev, 0, "bus", &busclk) == 0) {
		error = clk_enable(busclk);
		if (error != 0) {
			device_printf(dev, "cannot enable bus clock\n");
			return (error);
		}
	}

	/* A zero clock makes uart_bus_probe() fall back to uc_rclk. */
	error = uart_bus_probe(dev, (int)shift, (int)iowidth, (int)clock, 0, 0,
	    0);
	if (error > 0)
		return (error);

	/* XXX uart_bus_probe has changed the softc, so refresh it */
	sc = device_get_softc(dev);
	sc->baudclk = baudclk;
	sc->busclk = busclk;

	return (BUS_PROBE_VENDOR);
}

static int
mtk_uart_fdt_attach(device_t dev)
{
	phandle_t node;
	int error;

	error = uart_bus_attach(dev);
	if (error == 0) {
		node = ofw_bus_get_node(dev);
		OF_device_register_xref(OF_xref_from_node(node), dev);
	}

	return (error);
}

static int
mtk_uart_fdt_detach(device_t dev)
{
	struct mtk_uart_softc *sc;
	clk_t baudclk, busclk;
	int error;

	sc = device_get_softc(dev);
	baudclk = sc->baudclk;
	busclk = sc->busclk;

	error = uart_bus_detach(dev);
	if (error != 0)
		return (error);

	if (busclk != NULL)
		clk_release(busclk);
	if (baudclk != NULL)
		clk_release(baudclk);

	return (0);
}

static device_method_t mtk_uart_fdt_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mtk_uart_fdt_probe),
	DEVMETHOD(device_attach,	mtk_uart_fdt_attach),
	DEVMETHOD(device_detach,	mtk_uart_fdt_detach),
	DEVMETHOD_END
};

static driver_t mtk_uart_driver = {
	uart_driver_name,
	mtk_uart_fdt_methods,
	sizeof(struct mtk_uart_softc)
};

DRIVER_MODULE(uart_mtk, simplebus, mtk_uart_driver, 0, 0);
