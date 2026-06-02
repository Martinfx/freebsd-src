/*
* SPDX-License-Identifier: BSD-2-Clause
*
* Copyright (c) 2026 Martin Filla
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

#include <sys/param.h>
#include <sys/libkern.h>
#include <sys/kernel.h>
#include <sys/cdefs.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/fdt/simplebus.h>
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

static struct ofw_compat_data compat_data[] =
    {{"mediatek,mt7986a-pinctrl", 1},
{NULL,                      0}};

struct mt7986_pinmux_desc {
	const char *modes[8];
	bus_size_t reg_offset;
	int shift;
};

struct mt7986_functions_desc {
	const char *group;
	const char *function;
	const uint32_t pin;
	const char *mode;
};

struct mt7986_pinctrl_softc {
	device_t dev;
	struct resource *mem_res;
	const struct mt7986_pinmux_desc *pinmux;
	const struct mt7986_functions_desc *functions;
	int mem_rid;
};

/* for mt7986 from pinctrl-mt7986.yaml */
static const struct mt7986_functions_desc functions[] = {
	/* group, function, pin, mode */
	/* watchdog functions */
	{"watchdog", "watchdog", 0, "SYS_WATCHDOG"},

	/* led functions */
	{"wifi_led", "led", 1, NULL},
	{"wifi_led", "led", 2, NULL},

	/* i2c functions */
	{"i2c", "i2c", 3, NULL},
	{"i2c", "i2c", 4, NULL},

	/* uart functions */
	{"uart1_0", "uart", 7, NULL},
	{"uart1_0", "uart", 8, NULL},
	{"uart1_0", "uart", 9, NULL},
	{"uart1_0", "uart", 10, NULL},

	/* uart functions */
	{"uart1_rx_tx", "uart", 42, NULL},
	{"uart1_rx_tx", "uart", 43, NULL},

	/* uart functions */
	{"uart1_cts_rts", "uart", 44, NULL},
	{"uart1_cts_rts", "uart", 45, NULL},

	/* pcie functions */
	{"pcie_clk", "pcie", 9, NULL},
	{"pcie_wake", "pcie", 10, NULL},

	/* spi functions */
	{"spi1_0", "spi", 11, NULL},
	{"spi1_0", "spi", 12, NULL},
	{"spi1_0", "spi", 13, NULL},
	{"spi1_0", "spi", 14, NULL},

	/* pwm functions */
	{"pwm1_1", "pwm", 20, NULL},
	{"pwm0", "pwm", 21, "PWM0"},

	/* pwm functions */
	{"pwm1_0", "pwm", 22, "PWM1"},

	/* flash functions */
	{"snfi", "flash", 23, NULL},
	{"snfi", "flash", 24, NULL},
	{"snfi", "flash", 25, NULL},
	{"snfi", "flash", 26, NULL},
	{"snfi", "flash", 27, NULL},
	{"snfi", "flash", 28, NULL},

	/* spi functions */
	{"spi1_2", "spi", 29, NULL},
	{"spi1_2", "spi", 30, NULL},
	{"spi1_2", "spi", 31, NULL},
	{"spi1_2", "spi", 32, NULL},

	/* emmc functions */
	{"emmc_45", "emmc", 22, NULL},
	{"emmc_45", "emmc", 23, NULL},
	{"emmc_45", "emmc", 24, NULL},
	{"emmc_45", "emmc", 25, NULL},
	{"emmc_45", "emmc", 26, NULL},
	{"emmc_45", "emmc", 27, NULL},
	{"emmc_45", "emmc", 28, NULL},
	{"emmc_45", "emmc", 29, NULL},
	{"emmc_45", "emmc", 30, NULL},
	{"emmc_45", "emmc", 31, NULL},
	{"emmc_45", "emmc", 32, NULL},

	/* spi functions */
	{"spi1_1", "spi", 23, NULL},
	{"spi1_1", "spi", 24, NULL},
	{"spi1_1", "spi", 25, NULL},
	{"spi1_1", "spi", 26, NULL},

	/* uart functions */
	{"uart1_2_rx_tx", "uart", 29, NULL},
	{"uart1_2_rx_tx", "uart", 30, NULL},

	/* uart functions */
	{"uart1_2_cts_rts", "uart", 31, NULL},
	{"uart1_2_cts_rts", "uart", 32, NULL},

	/* uart functions */
	{"uart1_1", "uart", 23, NULL},
	{"uart1_1", "uart", 24, NULL},
	{"uart1_1", "uart", 25, NULL},
	{"uart1_1", "uart", 26, NULL},

	/* uart functions */
	{"uart2_0_rx_tx", "uart", 29, NULL},
	{"uart2_0_rx_tx", "uart", 30, NULL},

	/* uart functions */
	{"uart2_0_cts_rts", "uart", 31, NULL},
	{"uart2_0_cts_rts", "uart", 32, NULL},

	/* spi functions */
	{"spi0", "spi", 33, NULL},
	{"spi0", "spi", 34, NULL},
	{"spi0", "spi", 35, NULL},
	{"spi0", "spi", 36, NULL},

	/* spi functions */
	{"spi0_wp_hold", "spi", 37, NULL},
	{"spi0_wp_hold", "spi", 38, NULL},

	/* uart functions */
	{"uart1_3_rx_tx", "uart", 35, NULL},
	{"uart1_3_rx_tx", "uart", 36, NULL},

	/* uart functions */
	{"uart1_3_cts_rts", "uart", 37, NULL},
	{"uart1_3_cts_rts", "uart", 38, NULL},

	/* uart functions */
	{"uart2_1", "uart", 33, NULL},
	{"uart2_1", "uart", 34, NULL},
	{"uart2_1", "uart", 35, NULL},
	{"uart2_1", "uart", 36, NULL},

	/* spi functions */
	{"spi1_3", "spi", 33, NULL},
	{"spi1_3", "spi", 34, NULL},
	{"spi1_3", "spi", 35, NULL},
	{"spi1_3", "spi", 36, NULL},

	/* uart functions */
	{"uart0", "uart", 39, NULL},
	{"uart0", "uart", 40, NULL},

	/* pcie functions */
	{"pcie_pereset", "pcie", 41, NULL},

	/* uart functions */
	{"uart1", "uart", 42, "UART1_RXD"},
	{"uart1", "uart", 43, NULL},
	{"uart1", "uart", 44, NULL},
	{"uart1", "uart", 45, NULL},

	/* uart functions */
	{"uart2", "uart", 46, NULL},
	{"uart2", "uart", 47, NULL},
	{"uart2", "uart", 48, NULL},
	{"uart2", "uart", 49, NULL},

	/* emmc functions */
	{"emmc_51", "emmc", 50, "EMMC_DATA_0"},
	{"emmc_51", "emmc", 51, NULL},
	{"emmc_51", "emmc", 52, NULL},
	{"emmc_51", "emmc", 53, NULL},
	{"emmc_51", "emmc", 54, NULL},
	{"emmc_51", "emmc", 55, NULL},
	{"emmc_51", "emmc", 56, NULL},
	{"emmc_51", "emmc", 57, NULL},
	{"emmc_51", "emmc", 58, "EMMC_CMD"},
	{"emmc_51", "emmc", 59, NULL},
	{"emmc_51", "emmc", 60, NULL},
	{"emmc_51", "emmc", 61, "EMMC_RSTB"},

	/* audio functions */
	{"pcm", "audio", 62, NULL},
	{"pcm", "audio", 63, NULL},
	{"pcm", "audio", 64, NULL},
	{"pcm", "audio", 65, NULL},

	/* audio functions */
	{"i2s", "audio", 62, NULL},
	{"i2s", "audio", 63, NULL},
	{"i2s", "audio", 64, NULL},
	{"i2s", "audio", 65, NULL},

	{"switch_int", "eth", 66, NULL},
	{"mdc_mdio", "eth", 67, NULL},

	/* wifi functions */
	{"wf_2g", "wifi", 74, NULL},
	{"wf_2g", "wifi", 75, NULL},
	{"wf_2g", "wifi", 76, NULL},
	{"wf_2g", "wifi", 77, NULL},
	{"wf_2g", "wifi", 78, NULL},
	{"wf_2g", "wifi", 79, NULL},
	{"wf_2g", "wifi", 80, NULL},
	{"wf_2g", "wifi", 81, NULL},
	{"wf_2g", "wifi", 82, NULL},
	{"wf_2g", "wifi", 83, NULL},

	/* wifi functions */
	{"wf_5g", "wifi", 91, NULL},
	{"wf_5g", "wifi", 92, NULL},
	{"wf_5g", "wifi", 93, NULL},
	{"wf_5g", "wifi", 94, NULL},
	{"wf_5g", "wifi", 95, NULL},
	{"wf_5g", "wifi", 96, NULL},
	{"wf_5g", "wifi", 97, NULL},
	{"wf_5g", "wifi", 98, NULL},
	{"wf_5g", "wifi", 99, NULL},
	{"wf_5g", "wifi", 100, NULL},

	/* wifi functions */
	{"wf_dbdc", "wifi", 74, NULL},
	{"wf_dbdc", "wifi", 75, NULL},
	{"wf_dbdc", "wifi", 76, NULL},
	{"wf_dbdc", "wifi", 77, NULL},
	{"wf_dbdc", "wifi", 78, NULL},
	{"wf_dbdc", "wifi", 79, NULL},
	{"wf_dbdc", "wifi", 80, NULL},
	{"wf_dbdc", "wifi", 81, NULL},
	{"wf_dbdc", "wifi", 82, NULL},
	{"wf_dbdc", "wifi", 83, NULL},
	{"wf_dbdc", "wifi", 84, NULL},
	{"wf_dbdc", "wifi", 85, NULL},
};

static const struct mt7986_pinmux_desc pinmux[] = {
	/* from MT7986A_Reference_Manual_for_BPI-R3.pdf */
	/* number of pin , mode 0..7, register, shift */
	[42] = {{"GPIO7", "DRV_VBUS", "CONN0_UART_TXD0", "UART1_RXD", NULL, NULL, "DBG_MON_A6", NULL}, 0x300, 28},
	[11] = {{"GPIO6", "PCIE_PHY_I2C_SDA", "SGMII0_PHY_I2C_SDA", NULL, NULL, NULL, "DBG_MON_A5", NULL}, 0x300, 24},
	[10] = {{"GPIO5", "PCIE_PHY_I2C_SCL", "SGMII0_PHY_I2C_SCL", NULL, NULL, NULL, "DBG_MON_A4", NULL}, 0x300, 20},
	// XXX: Problem with pin 9
	[9] = {{"GPIO4", "I2C_SDA", "SGMII1_PHY_I2C_SDA", "U3_PHY_I2C_SDA", NULL, NULL, "DBG_MON_A3", NULL}, 0x300, 16},
	[8] = {{"GPIO3", "I2C_SCL", "SGMII1_PHY_I2C_SCL", "U3_PHY_I2C_SCL", NULL, NULL, "DBG_MON_A2", NULL}, 0x300, 12},
	[7] = {{"GPIO2", "WF5G_LED", NULL, NULL, NULL, NULL, "DBG_MON_A1", NULL}, 0x300, 8},
	[6] = {{"GPIO1", "WF2G_LED", NULL, NULL, NULL, NULL, "DBG_MON_A0", NULL}, 0x300, 4},
	[5] = {{"GPIO0", "SYS_WATCHDOG", NULL, NULL, NULL, NULL, NULL, NULL}, 0x300, 0},

	[20] = {{"GPIO15", "JTAG_JTRST_N", "WM0_JTAG_JTRST_N", NULL, NULL, "DFD_NTRST", NULL, NULL}, 0x310, 28},
	[19] = {{"GPIO14", "JTAG_JTCLK", "WM0_JTAG_JTCLK", "SPIC_CS", NULL, "DFD_TCK_XI", NULL, NULL}, 0x310, 24},
	[18] = {{"GPIO13", "JTAG_JTMS", "WM0_JTAG_JTMS", "SPIC_MISO", NULL, "DFD_TMS", NULL, NULL}, 0x310, 20},
	[17] = {{"GPIO12", "JTAG_JTDI", "WM0_JTAG_JTDI", "SPIC_MOSI", NULL, "DFD_TDI", NULL, NULL}, 0x310, 16},
	[16] = {{"GPIO11", "JTAG_JTDO", "WM0_JTAG_JTDO", "SPIC_CLK", NULL, "DFD_TDO", NULL, NULL}, 0x310, 12},
	[15] = {{"GPIO10", "PCIE_WAKE_N", NULL, "UART1_RTS", NULL, NULL, NULL, NULL}, 0x310, 8},
	[14] = {{"GPIO9", "PCIE_CLK_REQ", NULL, "UART1_CTS", NULL, NULL, NULL, NULL}, 0x310, 4},
	[13] = {{"GPIO8", "DRV_VBUS_1P", "CONN0_UART_TXD1", "UART1_TXD", NULL, NULL, "DBG_MON_A7", NULL}, 0x310, 0},

	[50] = {{"GPIO23", "SNFI_CLK", "EMMC_DATA_0", "SPIC_CLK", "UART1_RXD", NULL, NULL, NULL}, 0x320, 28},
	[61] = {{"GPIO22", "PWM1", "EMMC_RSTB", "NET_WO0_UART_TXD", "NET_WO1_UART_TXD", NULL, NULL, NULL}, 0x320, 24},
	[21] = {{"GPIO21", "PWM0", NULL, NULL, NULL, NULL, "DBG_MON_A13", NULL}, 0x320, 20},
	[22] = {{"GPIO20", "WO0_JTAG_JTRST_N", "PWM1", "WO1_JTAG_JTRST_N", NULL, "UDI_NTRST", "DBG_MON_A12", NULL}, 0x320, 16},
	//[x] = {{"GPIO19", "WO0_JTAG_JTCLK", "CONN_ICE1_1", "WO1_JTAG_JTCLK", NULL, "UDI_TCK_XI", "DBG_MON_A11", NULL}, 0x320, 12},
	//[x] = {{"GPIO18", "WO0_JTAG_JTMS", "CONN_ICE1_0", "WO1_JTAG_JTMS", NULL, "UDI_TMS", "DBG_MON_A10", NULL}, 0x320, 8},
	//[x] = {{"GPIO17", "WO0_JTAG_JTDI", "CONN_ICE0_1", "WO1_JTAG_JTDI", NULL, "UDI_TDI", "DBG_MON_A9", NULL}, 0x320, 4},
	//[x] = {{"GPIO16", "WO0_JTAG_JTDO", "CONN_ICE0_0", "WO1_JTAG_JTDO", NULL, "UDI_TDO", "DBG_MON_A8", NULL}, 0x320, 0},

	[58] = {{"GPIO31", "SPIC_MISO", "EMMC_CMD", "UART1_CTS", "UART2_CTS", NULL, NULL, NULL}, 0x330, 28},
	[57] = {{"GPIO30", "SPIC_MOSI", "EMMC_DATA_7", "UART1_TXD", "UART2_TXD", NULL, NULL, NULL}, 0x330, 24},
};

static void
mt7986_pinctrl_process_entry(struct mt7986_pinctrl_softc *sc, const char *group, char *function) {
	for (int i = 0; i < nitems(functions); i++) {
		if ((strcmp(functions[i].group, group) == 0) &&
		    (strcmp(functions[i].function, function) == 0)) {

			uint32_t pin = functions[i].pin;
			const char *mode = functions[i].mode;
			const struct mt7986_pinmux_desc *pinmux = &sc->pinmux[pin];
			if (mode != NULL && pinmux != NULL) {
				for (int j = 0; j < nitems(pinmux->modes); j++) {
					if(pinmux->modes[j] != NULL) {
						if (strcmp(pinmux->modes[j], mode) == 0) {
							uint32_t val = bus_read_4(sc->mem_res, pinmux->reg_offset);
							val &= ~(0xF << pinmux->shift);
							val |= (j << pinmux->shift);
							device_printf(sc->dev,
							    "Pin %d: reg 0x%lX (shift %d): mode %d, mask 0x%X, reg val 0x%08X\n",
							    pin,
							    pinmux->reg_offset,
							    pinmux->shift,
							    j,
							    0xF << pinmux->shift,
							    val
							);
							bus_write_4(sc->mem_res, pinmux->reg_offset, val);

							uint32_t check_val = bus_read_4(sc->mem_res, pinmux->reg_offset);
							uint32_t set_bits = (check_val >> pinmux->shift) & 0xF;
							if (set_bits != (j & 0xF)) {
								device_printf(sc->dev,
								    "Warning: Pin %d: mode not set correctly! Expected %d, got %d in reg 0x%X\n",
								    pin, j & 0xF, set_bits, check_val);
							} else {
								device_printf(sc->dev,
								    "Pin %d successfully set to mode %d, reg 0x%lX = 0x%08X\n",
								    pin, set_bits, pinmux->reg_offset, check_val);
							}

						}
					}
				}
			}
		}
	}
}

static int
mt7986_pinctrl_process_node(struct mt7986_pinctrl_softc *sc, phandle_t child) {
	char mux[64];
	const char **groups = NULL;
	char *function = NULL;
	int num_groups = 0;

	if (OF_getprop(child, "name", &mux, sizeof(mux)) > 0) {
		//device_printf(sc->dev, "mux: %s\n", mux);
		if (strncmp(mux, "mux", 3) == 0) {
			num_groups = ofw_bus_string_list_to_array(child, "groups", &groups);
			if (num_groups <= 0) {
				return (ENOENT);
			}

			if (OF_getprop_alloc(child, "function", (void **) &function) == -1) {
				OF_prop_free(groups);
				return (ENOENT);
			}

			for (int i = 0; i < num_groups; i++) {
				mt7986_pinctrl_process_entry(sc, groups[i], function);
			}
		}
	}
	OF_prop_free(groups);
	OF_prop_free(function);
	return 0;
}

static int
mt7986_pinctrl_configure(device_t dev, phandle_t cfgxref) {
	struct mt7986_pinctrl_softc *sc;
	phandle_t child, node;

	sc = device_get_softc(dev);
	node = OF_node_from_xref(cfgxref);

	for (child = OF_child(node); child != 0 && child != -1; child = OF_peer(child)) {
		mt7986_pinctrl_process_node(sc, child);
	}

	return 0;
}

static int
mt7986_pinctrl_probe(device_t dev) {
	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
		return (ENXIO);

	device_set_desc(dev, "Mediatek mt7986 pinctrl configuration");
	return (BUS_PROBE_DEFAULT);
}

static int
mt7986_pinctrl_attach(device_t dev) {
	struct mt7986_pinctrl_softc *sc;
	phandle_t node;

	sc = device_get_softc(dev);
	sc->dev = dev;
	node = ofw_bus_get_node(dev);

	sc->mem_rid = 0;
	if (ofw_bus_find_string_index(node, "reg-names", "gpio", &sc->mem_rid) != 0) {
		device_printf(dev, "Cannot find gpio register in reg-names\n");
		return (ENXIO);
	}

	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid, RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Warning: Could not map memory resource for gpio (rid %d)\n", sc->mem_rid);
		return (ENXIO);
	}

	sc->pinmux = pinmux;
	sc->functions = functions;

	fdt_pinctrl_register(dev, NULL);
	fdt_pinctrl_configure_tree(dev);

	return (0);
}

static int mt7986_pinctrl_detach(device_t dev) {
	struct mt7986_pinctrl_softc *sc;
	sc = device_get_softc(dev);

	if (sc->mem_res) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
	}

	return (0);
}

static device_method_t mt7986_pinctrl_methods[] = {
	DEVMETHOD(device_probe, mt7986_pinctrl_probe),
	DEVMETHOD(device_attach, mt7986_pinctrl_attach),
	DEVMETHOD(device_detach, mt7986_pinctrl_detach),
	DEVMETHOD(fdt_pinctrl_configure, mt7986_pinctrl_configure),
	DEVMETHOD_END
};

static DEFINE_CLASS_0(mt7986_pinctrl, mt7986_pinctrl_driver, mt7986_pinctrl_methods,
    sizeof(struct mt7986_pinctrl_softc));
EARLY_DRIVER_MODULE(mt7986_pinctrl, simplebus, mt7986_pinctrl_driver, NULL, NULL,
    71);