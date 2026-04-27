/*-
 * Copyright (c) 2026 Martin Filla <freebsd@sysctl.cz>
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

#define	MT_UART_HIGHS			0x09	/* highspeed mode select */
#define	MT_UART_SAMPLE_COUNT	0x0a	/* sample count (mode 3) */
#define	MT_UART_SAMPLE_POINT	0x0b	/* sample point (mode 3) */
#define MT_UART_RATE_FIX		0x0d

#define	MT_UART_HIGHS_NORMAL	0x0	/* 16x oversampling */
#define	MT_UART_HIGHS_HALF		0x1	/*  8x oversampling (bootloader only) */
#define	MT_UART_HIGHS_MID		0x2	/*  4x oversampling */
#define	MT_UART_HIGHS_FAST		0x3	/* programmable sample-count */

struct mt_softc {
	struct ns8250_softc ns8250_base;
	clk_t baud_clk;
	clk_t bus_clk;
};

static int
mt_uart_calc_divisor(uint32_t rclk, int *baudp, uint8_t *modep,
    uint16_t *quotp, uint8_t *samplecntp)
{
	uint32_t baud, quot, samples, actual, err;
	uint32_t best_quot = 0, best_samples = 0, best_err = UINT32_MAX;

	baud = (uint32_t)*baudp;
	if (baud == 0 || rclk == 0)
		return (EINVAL);

	/*
	 * Try mode 0 (16x oversampling) first — it's the canonical
	 * approach for rates up to 115200 per datasheet, and uses
	 * well-tested HW state. Accept it if error is within 2%.
	 */
	if (baud <= 115200) {
		quot = (rclk + 8 * baud) / (16 * baud);
		if (quot >= 1 && quot <= 0xffff) {
			actual = rclk / (16 * quot);
			err = (actual > baud) ? actual - baud : baud - actual;
			if (err * 50 < baud) {
				*modep = MT_UART_HIGHS_NORMAL;
				*quotp = (uint16_t)quot;
				*samplecntp = 0;
				*baudp = (int)actual;
				return (0);
			}
		}
	}

	/*
	 * Mode 3 sweep: variable sample_count gives best accuracy
	 * when fixed 16x oversampling can't hit the target.
	 */
	for (samples = 256; samples >= 3; samples--) {
		quot = (rclk + samples * baud / 2) / (samples * baud);
		if (quot < 1 || quot > 0xffff)
			continue;
		actual = rclk / (samples * quot);
		err = (actual >= baud) ? actual - baud : baud - actual;
		if (err < best_err) {
			best_err = err;
			best_samples = samples;
			best_quot = quot;
			if (err == 0)
				break;
		}
	}

	if (best_quot == 0)
		return (EINVAL);

	*modep = MT_UART_HIGHS_FAST;
	*quotp = (uint16_t)best_quot;
	*samplecntp = (uint8_t)(best_samples - 1);
	*baudp = (int)(rclk / (best_samples * best_quot));
	return (0);
}
/*
 * Program the baud rate into HW. The "park" sequence avoids a transient
 * state where DLL=1 (left over from mode 3) is still latched while HIGHS
 * is being changed. The MT7688 datasheet (page 156) warns that division
 * by 1 generates a BAUD signal that is constantly high — this would
 * freeze the TX shift register if it caught a character mid-flight.
 */
static int
mt_uart_set_baud(struct uart_bas *bas, int baudrate)
{
	uint16_t quot;
	uint8_t mode, sample_count, sample_point, lcr;
	int adj_baud, error;

	adj_baud = baudrate;
	error = mt_uart_calc_divisor(bas->rclk, &adj_baud, &mode, &quot,
	    &sample_count);
	if (error != 0)
		return (error);

	/* Ensure rate_fix is disabled — when set, TX/RX require f16m_en
	 * to be active externally, which we don't manage. */
	uart_setreg(bas, MT_UART_RATE_FIX, 0);
	uart_barrier(bas);

	lcr = uart_getreg(bas, REG_LCR);

	/* Step 1: park divisor at a safe high value (0xFFFF). */
	uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
	uart_barrier(bas);
	uart_setreg(bas, REG_DLL, 0xff);
	uart_setreg(bas, REG_DLH, 0xff);
	uart_barrier(bas);
	uart_setreg(bas, REG_LCR, lcr);
	uart_barrier(bas);

	/* Step 2: switch HIGHS mode while divisor is harmless. */
	uart_setreg(bas, MT_UART_HIGHS, mode);
	uart_barrier(bas);

	/* Step 3: program the real divisor for the new mode. */
	uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
	uart_barrier(bas);
	uart_setreg(bas, REG_DLL, quot & 0xff);
	uart_setreg(bas, REG_DLH, (quot >> 8) & 0xff);
	uart_barrier(bas);
	uart_setreg(bas, REG_LCR, lcr);
	uart_barrier(bas);

	/* Step 4: sample count + mid-point (datasheet page 159). */
	if (mode == MT_UART_HIGHS_FAST) {
		sample_point = sample_count / 2;
		uart_setreg(bas, MT_UART_SAMPLE_COUNT, sample_count);
		uart_setreg(bas, MT_UART_SAMPLE_POINT, sample_point);
	} else {
		/* Reset to defaults for non-mode-3. */
		uart_setreg(bas, MT_UART_SAMPLE_COUNT, 0x00);
		uart_setreg(bas, MT_UART_SAMPLE_POINT, 0xff);
	}
	uart_barrier(bas);

	return (0);
}

/*
 * Read back the baud rate currently programmed in HW. Used during probe
 * to recover the bootloader-configured rate so the tty layer can match
 * it. Returns 0 on invalid state.
 */
static int
mt_uart_get_baud(struct uart_bas *bas)
{
	uint32_t quot, divider, baud;
	uint8_t mode, lcr, sample_count;

	if (bas->rclk == 0)
		return (0);

	mode = uart_getreg(bas, MT_UART_HIGHS) & 0x3;

	lcr = uart_getreg(bas, REG_LCR);
	uart_setreg(bas, REG_LCR, lcr | LCR_DLAB);
	uart_barrier(bas);
	quot  = uart_getreg(bas, REG_DLL);
	quot |= (uint32_t)uart_getreg(bas, REG_DLH) << 8;
	uart_setreg(bas, REG_LCR, lcr);
	uart_barrier(bas);

	if (quot == 0)
		return (0);

	switch (mode) {
	case MT_UART_HIGHS_NORMAL:
		divider = 16;
		break;
	case MT_UART_HIGHS_HALF:
		divider = 8;
		break;
	case MT_UART_HIGHS_MID:
		divider = 4;
		break;
	case MT_UART_HIGHS_FAST:
		sample_count = uart_getreg(bas, MT_UART_SAMPLE_COUNT) & 0xff;
		divider = (uint32_t)sample_count + 1;
		break;
	default:
		return (0);
	}

	baud = bas->rclk / (divider * quot);
	return ((int)baud);
}

/* ------------------------------------------------------------------ */
/* Low-level operations (used during early console bring-up).         */
/* ------------------------------------------------------------------ */

static void
mt_ops_uart_init(struct uart_bas *bas, int baudrate, int databits,
    int stopbits, int parity)
{
	/* Ensure rate_fix is disabled — when set, TX/RX require f16m_en
	 * to be active externally, which we don't manage. */
	uart_setreg(bas, MT_UART_RATE_FIX, 0);
	uart_barrier(bas);

	/* Mask interrupts while we reprogram. */
	uart_setreg(bas, REG_IER, 0);
	uart_barrier(bas);

	switch (databits) {
	case 5:
		databits = LCR_5BITS;
		break;
	case 6:
		databits = LCR_6BITS;
		break;
	case 7:
		databits = LCR_7BITS;
		break;
	case 8:
		databits = LCR_8BITS;
		break;
	default:
		/* Unsupported */
		return;
	}


	uart_setreg(bas, REG_LCR, databits);
	uart_barrier(bas);

	if (bas->rclk && baudrate) {
		(void)mt_uart_set_baud(bas, baudrate);
		uart_barrier(bas);
	}

	/* Enable + flush FIFOs, set RX trigger to highest level. */
	uart_setreg(bas, REG_FCR,
	    FCR_ENABLE | FCR_XMT_RST | FCR_RCV_RST | FCR_RX_LOW);
	uart_barrier(bas);

	/* Clear pending status. */
	(void)uart_getreg(bas, REG_LSR);
	(void)uart_getreg(bas, REG_MSR);
	(void)uart_getreg(bas, REG_IIR);
	uart_barrier(bas);
}

/*
 * The standard 16550 low-level path. ns8250's static implementations of
 * these aren't exported by the FreeBSD uart framework (only the bus_*
 * methods are), so we provide our own. Functionally identical — the
 * data path doesn't depend on the highspeed register.
 */
static int
mt_ops_uart_probe(struct uart_bas *bas __unused)
{
	/* FDT match has already validated this is our HW. */
	return (0);
}

static void
mt_ops_uart_term(struct uart_bas *bas)
{
	uart_setreg(bas, REG_MCR, 0);
	uart_barrier(bas);
}

static void
mt_ops_uart_putc(struct uart_bas *bas, int c)
{
	int limit;

	limit = 250000;
	while ((uart_getreg(bas, REG_LSR) & LSR_THRE) == 0 && --limit)
		DELAY(4);
	uart_setreg(bas, REG_DATA, c);
	uart_barrier(bas);
}

static int
mt_ops_uart_rxready(struct uart_bas *bas)
{

	return ((uart_getreg(bas, REG_LSR) & LSR_RXRDY) != 0);
}

static int
mt_ops_uart_getc(struct uart_bas *bas, struct mtx *hwmtx)
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
	.probe   = mt_ops_uart_probe,
	.init    = mt_ops_uart_init,
	.term    = mt_ops_uart_term,
	.putc    = mt_ops_uart_putc,
	.rxready = mt_ops_uart_rxready,
	.getc    = mt_ops_uart_getc,
};

/* ------------------------------------------------------------------ */
/* Bus-attached methods.                                              */
/* ------------------------------------------------------------------ */

static int
mt_uart_bus_probe(struct uart_softc *sc)
{
	struct uart_bas *bas;
	int baud, error;

	bas = &sc->sc_bas;

	baud = mt_uart_get_baud(bas);

	error = ns8250_bus_probe(sc);
	if (error != 0)
		return (error);

	if (baud > 0 && sc->sc_sysdev != NULL &&
	    sc->sc_sysdev->baudrate == 0)
		sc->sc_sysdev->baudrate = baud;

	device_set_desc(sc->sc_dev, "MediaTek MT6577 UART");
	return (0);
}

static int
mt_uart_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{
	struct uart_bas *bas;
	int current_baud, diff;

	bas = &sc->sc_bas;

	/*
	 * Live reprogramming of the baud rate is currently unstable on
	 * the MT7622 console UART — changing the divisor while the SoC
	 * has the UART configured as console hangs the system. As a
	 * workaround we accept tcsetattr requests within 2% of the
	 * current rate (which covers the bootloader-set 115207 vs the
	 * userland-requested 115200 mismatch) and reject other changes.
	 */
	current_baud = mt_uart_get_baud(bas);
	if (current_baud > 0 && baudrate > 0) {
		diff = (current_baud > baudrate) ?
		    current_baud - baudrate : baudrate - current_baud;
		if (diff * 50 < current_baud)
			return (0);
	}

	return (ENOTSUP);
}

static kobj_method_t mt_uart_methods[] = {
	KOBJMETHOD(uart_probe,    mt_uart_bus_probe),
	KOBJMETHOD(uart_param,    mt_uart_bus_param),
	KOBJMETHOD(uart_attach,   ns8250_bus_attach),
	KOBJMETHOD(uart_detach,   ns8250_bus_detach),
	KOBJMETHOD(uart_flush,    ns8250_bus_flush),
	KOBJMETHOD(uart_getsig,   ns8250_bus_getsig),
	KOBJMETHOD(uart_ioctl,    ns8250_bus_ioctl),
	KOBJMETHOD(uart_ipend,    ns8250_bus_ipend),
	KOBJMETHOD(uart_receive,  ns8250_bus_receive),
	KOBJMETHOD(uart_setsig,   ns8250_bus_setsig),
	KOBJMETHOD(uart_transmit, ns8250_bus_transmit),
	KOBJMETHOD(uart_grab,     ns8250_bus_grab),
	KOBJMETHOD(uart_ungrab,   ns8250_bus_ungrab),
	KOBJMETHOD_END
};

/* ------------------------------------------------------------------ */
/* uart_class — drives both the console subsystem and the bus driver. */
/* ------------------------------------------------------------------ */

static struct uart_class mt_uart_class = {
	"mt_uart",
	mt_uart_methods,
	sizeof(struct mt_softc),
	.uc_ops      = &mt_uart_ops,
	.uc_range    = 0x100,
	.uc_rclk     = 0,
	.uc_rshift   = 2,
	.uc_riowidth = 4,
};

/* Compatible devices. */
static struct ofw_compat_data compat_data[] = {
	{"mediatek,mt6577-uart",(uintptr_t)&mt_uart_class},
	{ NULL,			(uintptr_t)NULL },
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
	shift = uart_fdt_get_shift1(node);

	rv = clk_get_by_ofw_name(dev, 0, "baud", &sc->baud_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot get 'baud' clock\n");
		return (ENXIO);
	}
	rv = clk_enable(sc->baud_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable baud clock: %d\n", rv);
		return (ENXIO);
	}

	rv = clk_get_by_ofw_name(dev, 0, "bus", &sc->bus_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot get 'bus' clock\n");
		return (ENXIO);
	}
	rv = clk_enable(sc->bus_clk);
	if (rv != 0) {
		device_printf(dev, "Cannot enable bus clock: %d\n", rv);
		return (ENXIO);
	}

	rv = clk_get_freq(sc->baud_clk, &freq);
	if (rv != 0) {
		device_printf(dev, "Cannot read baud clock freq: %d\n", rv);
		return (ENXIO);
	}

	rv = uart_bus_probe(dev, shift, 0, (int)freq, 0, 0, 0);
	if (rv > 0) {
		return (ENXIO);
	}

	return (rv);
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
	DEVMETHOD(device_probe,	 mt_uart_probe),
	DEVMETHOD(device_attach, uart_bus_attach),
	DEVMETHOD(device_detach, mt_uart_detach),
	DEVMETHOD_END
};

static driver_t mt_uart_driver = {
	uart_driver_name,
	mt_uart_bus_methods,
    	sizeof(struct mt_softc),
};

DRIVER_MODULE(mt_uart, simplebus,  mt_uart_driver, 0, 0);

