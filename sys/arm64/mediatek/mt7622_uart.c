/*
 * Copyright (c) 2025, 2026 Martin Filla <freebsd@sysctl.cz>
 * Copyright (c) 2025 Michal Meloun <mmel@FreeBSD.org>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/cdefs.h>

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

/* MediaTek specific registers (register indexes, scaled by reg-shift). */
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
    struct ns8250_softc	ns8250_base;
    clk_t			baud;
    clk_t			bus;
};

/*
 * Build the LCR value for the requested character format.  Identical to the
 * private ns8250_param() logic; replicated here because that symbol is not
 * exported and we have to program the divisor differently anyway.
 */
static uint8_t
mt_uart_lcr(int databits, int stopbits, int parity)
{
        uint8_t lcr;

        lcr = 0;
        if (databits >= 8)
                lcr |= LCR_8BITS;
        else if (databits == 7)
                lcr |= LCR_7BITS;
        else if (databits == 6)
                lcr |= LCR_6BITS;
        else
                lcr |= LCR_5BITS;
        if (stopbits > 1)
                lcr |= LCR_STOPB;
        lcr |= parity << 3;

        return (lcr);
}

/*
 * Program the baud rate generator.  The caller is responsible for serializing
 * access (holding sc_hwmtx or running single threaded during console init) and
 * must have already written the line control register, which is preserved here
 * across the divisor-latch access.
 *
 * baud = rclk / (oversampling * divisor), where the oversampling factor is
 * selected through the high speed register.  See MT7622 RM 1.11.
 */
static void
mt_uart_set_baud(struct uart_bas *bas, int baudrate)
{
        uint32_t divisor, samples;
        int highspeed;
        uint8_t lcr;

        if (baudrate <= 0 || bas->rclk == 0)
                return;

        /* Use the divisor based generator, not the fixed-rate override. */
        uart_setreg(bas, MTK_UART_RATE_FIX, 0);
        uart_barrier(bas);

        if (baudrate <= 115200) {
                highspeed = MTK_UART_HIGHS_16X;
                divisor = MTK_DIV_ROUND_CLOSEST(bas->rclk, 16 * baudrate);
        } else if (baudrate <= 576000) {
                highspeed = MTK_UART_HIGHS_4X;
                /* These two rates are not exactly representable; round down. */
                if (baudrate == 500000 || baudrate == 576000)
                        baudrate = 460800;
                divisor = howmany(bas->rclk, 4 * baudrate);
        } else {
                highspeed = MTK_UART_HIGHS_SAMPLE;
                divisor = howmany(bas->rclk, 256 * baudrate);
        }
        if (divisor == 0)
                divisor = 1;
        if (divisor > UART_DIV_MAX)
                divisor = UART_DIV_MAX;

        uart_setreg(bas, MTK_UART_HIGHS, highspeed);
        uart_barrier(bas);

        /* Program the divisor latch. */
        lcr = uart_getreg(bas, REG_LCR);
        uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
        uart_barrier(bas);
        uart_setreg(bas, REG_DLL, divisor & 0xff);
        uart_setreg(bas, REG_DLH, (divisor >> 8) & 0xff);
        uart_barrier(bas);
        uart_setreg(bas, REG_LCR, lcr);
        uart_barrier(bas);

        /*
         * Sample count/point only take effect in high speed mode 3, where
         * baud = rclk / ((sample_count + 1) * divisor).  Per the RM the sample
         * point sits in the middle of the bit: (sample_count - 1) / 2.  Leave
         * the registers at their reset values otherwise.
         */
        if (highspeed == MTK_UART_HIGHS_SAMPLE) {
                uint32_t denom = divisor * (uint32_t)baudrate;

                samples = denom != 0 ?
                          MTK_DIV_ROUND_CLOSEST((uint32_t)bas->rclk, denom) : 1;
                if (samples < 2)
                        samples = 2;
                uart_setreg(bas, MTK_UART_SAMPLE_COUNT, samples - 1);
                uart_setreg(bas, MTK_UART_SAMPLE_POINT, (samples - 2) >> 1);
        } else {
                uart_setreg(bas, MTK_UART_SAMPLE_COUNT, 0);
                uart_setreg(bas, MTK_UART_SAMPLE_POINT, 0xff);
        }
        uart_barrier(bas);
}

/*
 * Low-level (console) operations.  These are byte-for-byte equivalent to the
 * ns8250 implementations, replicated because those functions are private to
 * uart_dev_ns8250.c and we need a custom uart_ops just to override .init.
 */
static void
mt_uart_clrint(struct uart_bas *bas)
{
        uint8_t iir, lsr;

        iir = uart_getreg(bas, REG_IIR);
        while ((iir & IIR_NOPEND) == 0) {
                iir &= IIR_IMASK;
                if (iir == IIR_RLS) {
                        lsr = uart_getreg(bas, REG_LSR);
                        if (lsr & (LSR_BI | LSR_FE | LSR_PE))
                                (void)uart_getreg(bas, REG_DATA);
                } else if (iir == IIR_RXRDY || iir == IIR_RXTOUT)
                        (void)uart_getreg(bas, REG_DATA);
                else if (iir == IIR_MLSC)
                        (void)uart_getreg(bas, REG_MSR);
                uart_barrier(bas);
                iir = uart_getreg(bas, REG_IIR);
        }
}

static int
mt_uart_probe_low(struct uart_bas *bas)
{
        uint8_t val;

        /* Check known 0 bits that don't depend on DLAB. */
        val = uart_getreg(bas, REG_IIR);
        if (val & 0x30)
                return (ENXIO);
        val = uart_getreg(bas, REG_MCR);
        if (val & 0xa0)
                return (ENXIO);

        return (0);
}

static void
mt_uart_init_low(struct uart_bas *bas, int baudrate, int databits,
                 int stopbits, int parity)
{
        uint8_t ier;

        /* Disable all interrupt sources. */
        ier = uart_getreg(bas, REG_IER) & 0xe0;
        uart_setreg(bas, REG_IER, ier);
        uart_barrier(bas);

        /* Program the character format. */
        uart_setreg(bas, REG_LCR, mt_uart_lcr(databits, stopbits, parity));
        uart_barrier(bas);

        /*
         * Only (re)program the baud rate when the reference clock is known.
         * As a system console the hardware is brought up before the clock
         * framework is available, so bas->rclk is still 0 here; in that case
         * keep the divisor and high speed settings the boot loader configured
         * (they are known good, the console was just used) until mt_uart_param()
         * reprograms the hardware with the real rate fetched in mt_uart_probe().
         */
        if (baudrate > 0 && bas->rclk != 0)
                mt_uart_set_baud(bas, baudrate);

        /* Disable the FIFO. */
        uart_setreg(bas, REG_FCR, 0);
        uart_barrier(bas);
        /* Set RTS & DTR. */
        uart_setreg(bas, REG_MCR, MCR_IE | MCR_RTS | MCR_DTR);
        uart_barrier(bas);

        mt_uart_clrint(bas);
}

static void
mt_uart_term_low(struct uart_bas *bas)
{

        /* Clear RTS & DTR. */
        uart_setreg(bas, REG_MCR, MCR_IE);
        uart_barrier(bas);
}

static void
mt_uart_putc_low(struct uart_bas *bas, int c)
{
        int limit;

        limit = 250000;
        while ((uart_getreg(bas, REG_LSR) & LSR_THRE) == 0 && --limit)
                DELAY(4);
        uart_setreg(bas, REG_DATA, c);
        uart_barrier(bas);
}

static int
mt_uart_rxready_low(struct uart_bas *bas)
{

        return ((uart_getreg(bas, REG_LSR) & LSR_RXRDY) != 0 ? 1 : 0);
}

static int
mt_uart_getc_low(struct uart_bas *bas, struct mtx *hwmtx)
{
        int c;

        uart_lock(hwmtx);
        while ((uart_getreg(bas, REG_LSR) & LSR_RXRDY) == 0) {
                uart_unlock(hwmtx);
                DELAY(4);
                uart_lock(hwmtx);
        }
        c = uart_getreg(bas, REG_DATA);
        uart_unlock(hwmtx);

        return (c);
}

static struct uart_ops mt_uart_ops = {
    .probe = mt_uart_probe_low,
    .init = mt_uart_init_low,
    .term = mt_uart_term_low,
    .putc = mt_uart_putc_low,
    .rxready = mt_uart_rxready_low,
    .getc = mt_uart_getc_low,
};

/*
 * High-level (bus) operations.  Everything is plain ns8250 except baud rate
 * programming, which must honour the high speed register.
 */
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

        return (0);
}

/*
 * Read back and report the currently programmed baud rate.  Diagnostic only,
 * used under bootverbose during bring-up to show what the boot loader left in
 * the hardware versus the reference clock the driver sees.  Formulas per the
 * MT7622 Reference Manual 1.11:
 *   highspeed 0/1/2: baud = rclk / (16|8|4) / divisor
 *   highspeed 3:     baud = rclk / ((sample_count + 1) * divisor)
 */
static void
mt_uart_dump_baud(struct uart_softc *sc)
{
        struct uart_bas *bas = &sc->sc_bas;
        uint32_t divisor, baud;
        uint8_t lcr, highs, sample_count;

        highs = uart_getreg(bas, MTK_UART_HIGHS) & 0x3;
        sample_count = uart_getreg(bas, MTK_UART_SAMPLE_COUNT);

        lcr = uart_getreg(bas, REG_LCR);
        uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
        uart_barrier(bas);
        divisor = (uart_getreg(bas, REG_DLH) << 8) | uart_getreg(bas, REG_DLL);
        uart_setreg(bas, REG_LCR, lcr);
        uart_barrier(bas);

        switch (highs) {
                case MTK_UART_HIGHS_16X:
                        baud = divisor != 0 ? bas->rclk / 16 / divisor : 0;
                        break;
                case MTK_UART_HIGHS_8X:
                        baud = divisor != 0 ? bas->rclk / 8 / divisor : 0;
                        break;
                case MTK_UART_HIGHS_4X:
                        baud = divisor != 0 ? bas->rclk / 4 / divisor : 0;
                        break;
                default: /* MTK_UART_HIGHS_SAMPLE */
                        baud = divisor != 0 ?
                               bas->rclk / ((sample_count + 1) * divisor) : 0;
                        break;
        }

        device_printf(sc->sc_dev,
            "rclk=%u highspeed=%u divisor=%u sample_count=%u baud=%u\n",
            bas->rclk, highs, divisor, sample_count, baud);
}

static int
mt_uart_attach(struct uart_softc *sc)
{
        struct uart_bas *bas = &sc->sc_bas;
        int rv;

        rv = ns8250_bus_attach(sc);
        if (rv != 0)
                return (rv);

        /* Use the divisor based baud generator, not the fixed-rate override. */
        uart_setreg(bas, MTK_UART_RATE_FIX, 0);
        uart_barrier(bas);

        if (bootverbose)
                mt_uart_dump_baud(sc);

        return (0);
}

static kobj_method_t mt_methods[] = {
    KOBJMETHOD(uart_probe,		ns8250_bus_probe),
    KOBJMETHOD(uart_attach,		mt_uart_attach),
    KOBJMETHOD(uart_detach,		ns8250_bus_detach),
    KOBJMETHOD(uart_flush,		ns8250_bus_flush),
    KOBJMETHOD(uart_getsig,		ns8250_bus_getsig),
    KOBJMETHOD(uart_ioctl,		ns8250_bus_ioctl),
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

static struct uart_class mt_uart_class = {
    "mediatek",			/* name */
    mt_methods,			/* methods */
    sizeof(struct mt_softc),	/* size */
    .uc_ops = &mt_uart_ops,
    .uc_range = 0x100,
    .uc_rclk = 0,			/* Supplied at probe time from the clock. */
    .uc_rshift = 2,
    .uc_riowidth = 4,
};

/* Compatible devices. */
static struct ofw_compat_data compat_data[] = {
    {"mediatek,mt7622-uart",	(uintptr_t)&mt_uart_class},
    {"mediatek,mt6577-uart",	(uintptr_t)&mt_uart_class},
    {NULL,				(uintptr_t)NULL},
};

UART_FDT_CLASS(compat_data);

/*
 * UART Driver interface.
 */
static int
mt_uart_get_shift(phandle_t node)
{
        pcell_t shift;

        if (OF_getencprop(node, "reg-shift", &shift, sizeof(shift)) <= 0)
                shift = 2;
        return ((int)shift);
}

static int
mt_uart_probe(device_t dev)
{
        struct mt_softc *sc;
        phandle_t node;
        uint64_t freq;
        int shift, rv;
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

        rv = clk_get_by_ofw_name(dev, 0, "baud", &sc->baud);
        if (rv != 0) {
                device_printf(dev, "Cannot get 'baud' clock: %d\n", rv);
                return (ENXIO);
        }
        rv = clk_get_by_ofw_name(dev, 0, "bus", &sc->bus);
        if (rv != 0) {
                device_printf(dev, "Cannot get 'bus' clock: %d\n", rv);
                goto fail_baud;
        }
        rv = clk_enable(sc->baud);
        if (rv != 0) {
                device_printf(dev, "Cannot enable 'baud' clock: %d\n", rv);
                goto fail_bus;
        }
        rv = clk_enable(sc->bus);
        if (rv != 0) {
                device_printf(dev, "Cannot enable 'bus' clock: %d\n", rv);
                goto fail_baud_dis;
        }
        rv = clk_get_freq(sc->baud, &freq);
        if (rv != 0 || freq == 0) {
                device_printf(dev, "Cannot get 'baud' clock frequency: %d\n", rv);
                goto fail_bus_dis;
        }

        rv = uart_bus_probe(dev, shift, 0, (int)freq, 0, 0, 0);
        if (rv != 0)
                goto fail_bus_dis;

        return (0);

        fail_bus_dis:
        clk_disable(sc->bus);
        fail_baud_dis:
        clk_disable(sc->baud);
        fail_bus:
        clk_release(sc->bus);
        sc->bus = NULL;
        fail_baud:
        clk_release(sc->baud);
        sc->baud = NULL;
        return (ENXIO);
}

static int
mt_uart_detach(device_t dev)
{
        struct mt_softc *sc;
        int rv;

        sc = device_get_softc(dev);

        rv = uart_bus_detach(dev);
        if (rv != 0)
                return (rv);

        if (sc->bus != NULL) {
                clk_disable(sc->bus);
                clk_release(sc->bus);
                sc->bus = NULL;
        }
        if (sc->baud != NULL) {
                clk_disable(sc->baud);
                clk_release(sc->baud);
                sc->baud = NULL;
        }

        return (0);
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

DRIVER_MODULE(mt_uart, simplebus, mt_uart_driver, 0, 0);