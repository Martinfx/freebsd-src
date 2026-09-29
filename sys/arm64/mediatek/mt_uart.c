/*
 * Copyright (c) 2025, 2026 Martin Filla <freebsd@sysctl.cz>
 * SPDX-License-Identifier: BSD-2-Clause
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

#define	MTK_UART_HIGHS		0x09	/* High speed mode (oversampling) select */
#define	MTK_UART_SAMPLE_COUNT	0x0a	/* Sample count, high speed mode 3 only */
#define	MTK_UART_SAMPLE_POINT	0x0b	/* Sample point, high speed mode 3 only */
#define	MTK_UART_RATE_FIX	0x0d	/* Fixed-rate override register */

#define	MTK_UART_HIGHS_16X	0	/* baud = rclk / 16 / divisor */
#define	MTK_UART_HIGHS_8X	1	/* baud = rclk /  8 / divisor */
#define	MTK_UART_HIGHS_4X	2	/* baud = rclk /  4 / divisor */
#define	MTK_UART_HIGHS_SAMPLE	3	/* baud = rclk / (sample_count+1) / div */

#define	UART_DIV_MAX		0xffff	/* Divisor latch is 16 bit. */

#define	MTK_DIV_ROUND_CLOSEST(n, d)	(((n) + (d) / 2) / (d))

struct mt_softc {
	struct ns8250_softc 	ns8250_base;
	clk_t			baud_clk;
	clk_t			bus_clk;
};

static uint8_t
mt_uart_lcr(int databits, int stopbits, int parity)
{
        uint8_t lcr;

        lcr = 0;
        if (databits >= 8) {
                lcr |= LCR_8BITS;
        }
        else if (databits == 7) {
                lcr |= LCR_7BITS;
        }
        else if (databits == 6) {
                lcr |= LCR_6BITS;
        }
        else {
                lcr |= LCR_5BITS;
        }

        if (stopbits > 1) {
                lcr |= LCR_STOPB;
        }

        lcr |= parity << 3;

        return (lcr);
}

static uint32_t
mt_uart_calc_baud(u_int rclk, uint8_t highs, uint32_t divisor,
                  uint8_t sample_count)
{
        if (divisor == 0)
                return (0);
        switch (highs & 0x3) {
                case MTK_UART_HIGHS_16X:
                        return (rclk / 16 / divisor);
                case MTK_UART_HIGHS_8X:
                        return (rclk / 8 / divisor);
                case MTK_UART_HIGHS_4X:
                        return (rclk / 4 / divisor);
                default: /* MTK_UART_HIGHS_SAMPLE */
                        return (rclk / ((sample_count + 1) * divisor));
        }
}

static int
mt_uart_get_baud(struct uart_bas *bas)
{
        uint32_t divisor;
        uint8_t highs, lcr, sample_count;

        if (bas->rclk == 0)
                return (0);

        lcr = uart_getreg(bas, REG_LCR);
        uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
        uart_barrier(bas);
        divisor = uart_getreg(bas, REG_DLL) |
            (uart_getreg(bas, REG_DLH) << 8);
        uart_barrier(bas);
        uart_setreg(bas, REG_LCR, lcr);
        uart_barrier(bas);

        highs = uart_getreg(bas, MTK_UART_HIGHS);
        sample_count = uart_getreg(bas, MTK_UART_SAMPLE_COUNT);

        return (mt_uart_calc_baud(bas->rclk, highs, divisor, sample_count));
}

static void
mt_uart_set_baud(struct uart_bas *bas, int baudrate)
{
        uint32_t divisor, samples;
        uint8_t lcr;

        if (baudrate <= 0 || bas->rclk == 0)
                return;

        uart_setreg(bas, MTK_UART_RATE_FIX, 0);
        uart_barrier(bas);

        divisor = howmany(bas->rclk, (uint32_t)baudrate * 256);
        if (divisor == 0)
                divisor = 1;
        if (divisor > UART_DIV_MAX)
                divisor = UART_DIV_MAX;

        samples = MTK_DIV_ROUND_CLOSEST(bas->rclk, divisor * (uint32_t)baudrate);
        if (samples < 2)
                samples = 2;
        if (samples > 256)
                samples = 256;

        uart_setreg(bas, MTK_UART_HIGHS, MTK_UART_HIGHS_SAMPLE);
        uart_barrier(bas);

        lcr = uart_getreg(bas, REG_LCR);
        uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
        uart_barrier(bas);
        uart_setreg(bas, REG_DLL, divisor & 0xff);
        uart_setreg(bas, REG_DLH, (divisor >> 8) & 0xff);
        uart_barrier(bas);
        uart_setreg(bas, REG_LCR, lcr);
        uart_barrier(bas);
        uart_setreg(bas, MTK_UART_SAMPLE_COUNT, samples - 1);
        uart_setreg(bas, MTK_UART_SAMPLE_POINT, (samples - 2) >> 1);
        uart_barrier(bas);
}

static int
mt_uart_param(struct uart_softc *sc, int baudrate, int databits, int stopbits,
              int parity)
{
        struct uart_bas *bas = &sc->sc_bas;

        uart_lock(sc->sc_hwmtx);
        uart_setreg(bas, REG_LCR, mt_uart_lcr(databits, stopbits, parity));
        uart_barrier(bas);
        mt_uart_set_baud(bas, baudrate);
        uart_unlock(sc->sc_hwmtx);

        if (bootverbose) {
                device_printf(sc->sc_dev, "set baud %d %d %d\n", baudrate,
                    databits, stopbits);
        }

        return (0);
}

static int
mt_uart_ioctl(struct uart_softc *sc, int request, intptr_t data)
{
        int baudrate;

        if (request != UART_IOCTL_BAUD)
                return (ns8250_bus_ioctl(sc, request, data));

        uart_lock(sc->sc_hwmtx);
        baudrate = mt_uart_get_baud(&sc->sc_bas);
        uart_unlock(sc->sc_hwmtx);

        if (baudrate <= 0)
                return (ENXIO);
        *(int *)data = baudrate;
        return (0);
}

static int
mt_uart_attach(struct uart_softc *sc)
{
        struct uart_bas *bas = &sc->sc_bas;
        int rv;

        rv = ns8250_bus_attach(sc);
        if (rv != 0) {
                return (rv);
        }

        uart_setreg(bas, MTK_UART_RATE_FIX, 0);
        uart_barrier(bas);

	return (0);
}

static kobj_method_t mt_methods[] = {
        KOBJMETHOD(uart_probe,		ns8250_bus_probe),
        KOBJMETHOD(uart_attach,		mt_uart_attach),
        KOBJMETHOD(uart_detach,		ns8250_bus_detach),
        KOBJMETHOD(uart_flush,		ns8250_bus_flush),
        KOBJMETHOD(uart_getsig,		ns8250_bus_getsig),
        KOBJMETHOD(uart_ioctl,		mt_uart_ioctl),
        KOBJMETHOD(uart_ipend,		ns8250_bus_ipend),
        KOBJMETHOD(uart_param,		mt_uart_param),
        KOBJMETHOD(uart_receive,	ns8250_bus_receive),
        KOBJMETHOD(uart_setsig,		ns8250_bus_setsig),
        KOBJMETHOD(uart_transmit,	ns8250_bus_transmit),
        KOBJMETHOD(uart_txbusy,		ns8250_bus_txbusy),
        KOBJMETHOD(uart_grab,		ns8250_bus_grab),
        KOBJMETHOD(uart_ungrab,		ns8250_bus_ungrab),
        KOBJMETHOD_END
};


static int
mt_uart_probe_bas(struct uart_bas *bas)
{
        return (uart_ns8250_ops.probe(bas));
}

static void
mt_uart_init(struct uart_bas *bas, int baudrate, int databits, int stopbits,
             int parity)
{
        uart_ns8250_ops.init(bas, 0, databits, stopbits, parity);
        mt_uart_set_baud(bas, baudrate);
}

static void
mt_uart_term(struct uart_bas *bas)
{
        uart_ns8250_ops.term(bas);
}

static void
mt_uart_putc(struct uart_bas *bas, int c)
{
        uart_ns8250_ops.putc(bas, c);
}

static int
mt_uart_rxready(struct uart_bas *bas)
{
        return (uart_ns8250_ops.rxready(bas));
}

static int
mt_uart_getc(struct uart_bas *bas, struct mtx *hwmtx)
{
        return (uart_ns8250_ops.getc(bas, hwmtx));
}

static struct uart_ops mt_uart_ops = {
        .probe = mt_uart_probe_bas,
        .init = mt_uart_init,
        .term = mt_uart_term,
        .putc = mt_uart_putc,
        .rxready = mt_uart_rxready,
        .getc = mt_uart_getc,
};

static struct uart_class mt_uart_class = {
	"mediatek class",
	mt_methods,
    	sizeof(struct mt_softc),
	.uc_ops = &mt_uart_ops,
	.uc_range = 8,
	.uc_rclk = 0,
	.uc_rshift = 2,
	.uc_riowidth = 4,
};

/* Compatible devices. */
static struct ofw_compat_data compat_data[] = {
        {"mediatek,mt6577-uart", (uintptr_t) &mt_uart_class},
        {NULL,  (uintptr_t) NULL},
};

UART_FDT_CLASS(compat_data);

/*
 * UART Driver interface.
 */
static int mt_uart_get_shift(phandle_t node)
{
        pcell_t shift;

        if ((OF_getencprop(node, "reg-shift", &shift, sizeof(shift))) <= 0)
        {
                shift = 2;
        }

        return ((int) shift);
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
	shift = mt_uart_get_shift(node);
	rv = clk_get_by_ofw_name(dev, 0, "baud", &sc->baud_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot get UART baud clock: %d\n", rv);
		return (ENXIO);
	}
	rv = clk_enable(sc->baud_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable UART baud clock: %d\n", rv);
		return (ENXIO);
	}

	rv = clk_get_by_ofw_name(dev, 0, "bus", &sc->bus_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot get UART bus clock: %d\n", rv);
		return (ENXIO);
	}
	rv = clk_enable(sc->bus_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable UART bus clock: %d\n", rv);
		return (ENXIO);
	}

	rv = clk_get_freq(sc->baud_clk, &freq);
	if (rv != 0) {
		device_printf(dev, "Cannot get freq UART clock: %d\n", rv);
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
