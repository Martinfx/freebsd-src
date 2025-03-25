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
    {{"mediatek,mt7622-pinctrl", 1}, {NULL, 0}};

struct mt7622_group_desc {
    const char *name;
    const uint32_t *pins;
    size_t npins;
};

struct mt7622_function_desc {
    const char *name;
    const struct mt7622_group_desc *groups;
    size_t ngroups;
};

#define GROUP(name, pins) \
    { name, pins, nitems(pins) }

/*static const struct mt7622_group_desc emmc_groups[] = {
        GROUP("emmc", emmc_pins),
        GROUP("emmc_rst", emmc_rst_pins),
};*/

/* emmc groups */
static const unsigned int emmc_pins[] = {40, 41, 42, 43, 44, 45, 47, 48, 49, 50};
static const unsigned int emmc_rst_pins[] = {37};
static const struct mt7622_group_desc emmc_groups[] = {
        {"emmc",     emmc_pins,     nitems(emmc_pins)},
        {"emmc_rst", emmc_rst_pins, nitems(emmc_rst_pins)},
};

/* eth groups */
static const unsigned int esw_pins[] = {51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
static const unsigned int esw_p0_p1_pins[] = {51, 52, 53, 54, 55, 56, 57, 58};
static const unsigned int esw_p2_p3_p4_pins[] = {59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
static const unsigned int rgmii_via_esw_pins[] = {59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
static const unsigned int rgmii_via_gmac1_pins[] = {59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
static const unsigned int rgmii_via_gmac2_pins[] = {25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};
static const unsigned int mdc_mdio_pins[] = {23, 24};
static const struct mt7622_group_desc eth_groups[] = {
        { "esw", esw_pins, nitems(esw_pins) },
        { "esw_p0_p1", esw_p0_p1_pins, nitems(esw_p0_p1_pins) },
        { "esw_p2_p3_p4", esw_p2_p3_p4_pins, nitems(esw_p2_p3_p4_pins) },
        { "rgmii_via_esw", rgmii_via_esw_pins, nitems(rgmii_via_esw_pins) },
        { "rgmii_via_gmac1", rgmii_via_gmac1_pins, nitems(rgmii_via_gmac1_pins) },
        { "rgmii_via_gmac2", rgmii_via_gmac2_pins, nitems(rgmii_via_gmac2_pins) },
        { "mdc_mdio", mdc_mdio_pins, nitems(mdc_mdio_pins) },
};

/* i2c groups */
static const unsigned int i2c0_pins[] = {14, 15};
static const unsigned int i2c1_0_pins[] = {55, 56};
static const unsigned int i2c1_1_pins[] = {73, 74};
static const unsigned int i2c1_2_pins[] = {87, 88};
static const unsigned int i2c2_0_pins[] = {57, 58};
static const unsigned int i2c2_1_pins[] = {75, 76};
static const unsigned int i2c2_2_pins[] = {89, 90};
static const unsigned int i2s_in_mclk_bclk_ws_pins[] = {3, 4, 5};
static const unsigned int i2s1_in_data_pins[] = {1};
static const unsigned int i2s2_in_data_pins[] = {16};
static const unsigned int i2s3_in_data_pins[] = {17};
static const unsigned int i2s4_in_data_pins[] = {18};
static const unsigned int i2s_out_mclk_bclk_ws_pins[] = {3, 4, 5};
static const unsigned int i2s1_out_data_pins[] = {2};
static const unsigned int i2s2_out_data_pins[] = {19};
static const unsigned int i2s3_out_data_pins[] = {20};
static const unsigned int i2s4_out_data_pins[] = {21};
static const struct mt7622_group_desc i2c_groups[] = {
        { "i2c0", i2c0_pins, nitems(i2c0_pins) },
        { "i2c1_0", i2c1_0_pins, nitems(i2c1_0_pins) },
        { "i2c1_1", i2c1_1_pins, nitems(i2c1_1_pins) },
        { "i2c1_2", i2c1_2_pins, nitems(i2c1_2_pins) },
        { "i2c2_0", i2c2_0_pins, nitems(i2c2_0_pins) },
        { "i2c2_1", i2c2_1_pins, nitems(i2c2_1_pins) },
        { "i2c2_2", i2c2_2_pins, nitems(i2c2_2_pins) },
        { "i2s_in_mclk_bclk_ws", i2s_in_mclk_bclk_ws_pins, nitems(i2s_in_mclk_bclk_ws_pins) },
        { "i2s1_in_data", i2s1_in_data_pins, nitems(i2s1_in_data_pins) },
        { "i2s2_in_data", i2s2_in_data_pins, nitems(i2s2_in_data_pins) },
        { "i2s3_in_data", i2s3_in_data_pins, nitems(i2s3_in_data_pins) },
        { "i2s4_in_data", i2s4_in_data_pins, nitems(i2s4_in_data_pins) },
        { "i2s_out_mclk_bclk_ws", i2s_out_mclk_bclk_ws_pins, nitems(i2s_out_mclk_bclk_ws_pins) },
        { "i2s1_out_data", i2s1_out_data_pins, nitems(i2s1_out_data_pins) },
        { "i2s2_out_data", i2s2_out_data_pins, nitems(i2s2_out_data_pins) },
        { "i2s3_out_data", i2s3_out_data_pins, nitems(i2s3_out_data_pins) },
        { "i2s4_out_data", i2s4_out_data_pins, nitems(i2s4_out_data_pins) },
};

/* ir groups */
static const unsigned int ir_0_tx_pins[] = {16};
static const unsigned int ir_1_tx_pins[] = {59};
static const unsigned int ir_2_tx_pins[] = {99};
static const unsigned int ir_0_rx_pins[] = {17};
static const unsigned int ir_1_rx_pins[] = {60};
static const unsigned int ir_2_rx_pins[] = {100};
static const struct mt7622_group_desc ir_groups[] = {
        { "ir_0_tx", ir_0_tx_pins, nitems(ir_0_tx_pins) },
        { "ir_1_tx", ir_1_tx_pins, nitems(ir_1_tx_pins) },
        { "ir_2_tx", ir_2_tx_pins, nitems(ir_2_tx_pins) },
        { "ir_0_rx", ir_0_rx_pins, nitems(ir_0_rx_pins) },
        { "ir_1_rx", ir_1_rx_pins, nitems(ir_1_rx_pins) },
        { "ir_2_rx", ir_2_rx_pins, nitems(ir_2_rx_pins) },
};

/* led groups */
static const unsigned int ephy_leds_pins[] = {86, 91, 92, 93, 94};
static const unsigned int ephy0_led_pins[] = {86};
static const unsigned int ephy1_led_pins[] = {91};
static const unsigned int ephy2_led_pins[] = {92};
static const unsigned int ephy3_led_pins[] = {93};
static const unsigned int ephy4_led_pins[] = {94};
static const unsigned int wled_pins[] = {85};
static const struct mt7622_group_desc led_groups[] = {
        { "ephy_leds", ephy_leds_pins, nitems(ephy_leds_pins) },
        { "ephy0_led", ephy0_led_pins, nitems(ephy0_led_pins) },
        { "ephy1_led", ephy1_led_pins, nitems(ephy1_led_pins) },
        { "ephy2_led", ephy2_led_pins, nitems(ephy2_led_pins) },
        { "ephy3_led", ephy3_led_pins, nitems(ephy3_led_pins) },
        { "ephy4_led", ephy4_led_pins, nitems(ephy4_led_pins) },
        { "wled", wled_pins, nitems(wled_pins) },
};

/* flash groups */
static const unsigned int par_nand_pins[] = {37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50};
static const unsigned int snfi_pins[] = {8, 9, 10, 11, 12, 13};
static const unsigned int spi_nor_pins[] = {8, 9, 10, 11, 12, 13};
static const struct mt7622_group_desc flash_groups[] = {
        { "par_nand", par_nand_pins, nitems(par_nand_pins) },
        { "snfi", snfi_pins, nitems(snfi_pins) },
        { "spi_nor", spi_nor_pins, nitems(spi_nor_pins) },
};

/* pcie groups */
static const unsigned int pcie0_0_waken_pins[] = {14};
static const unsigned int pcie0_1_waken_pins[] = {79};
static const unsigned int pcie1_0_waken_pins[] = {14};
static const unsigned int pcie0_0_clkreq_pins[] = {15};
static const unsigned int pcie0_1_clkreq_pins[] = {80};
static const unsigned int pcie1_0_clkreq_pins[] = {15};
static const unsigned int pcie0_pad_perst_pins[] = {83};
static const unsigned int pcie1_pad_perst_pins[] = {84};
static const struct mt7622_group_desc pcie_groups[] = {
        { "pcie0_0_waken", pcie0_0_waken_pins, nitems(pcie0_0_waken_pins) },
        { "pcie0_1_waken", pcie0_1_waken_pins, nitems(pcie0_1_waken_pins) },
        { "pcie1_0_waken", pcie1_0_waken_pins, nitems(pcie1_0_waken_pins) },
        { "pcie0_0_clkreq", pcie0_0_clkreq_pins, nitems(pcie0_0_clkreq_pins) },
        { "pcie0_1_clkreq", pcie0_1_clkreq_pins, nitems(pcie0_1_clkreq_pins) },
        { "pcie1_0_clkreq", pcie1_0_clkreq_pins, nitems(pcie1_0_clkreq_pins) },
        { "pcie0_pad_perst", pcie0_pad_perst_pins, nitems(pcie0_pad_perst_pins) },
        { "pcie1_pad_perst", pcie1_pad_perst_pins, nitems(pcie1_pad_perst_pins) },
};

/* pmic groups */
static const unsigned int pmic_bus_pins[] = {71, 72};
static const struct mt7622_group_desc pmic_groups[] = {
        {"pmic_bus", pmic_bus_pins, nitems(pmic_bus_pins)},
};

/* pwm groups */
static const unsigned int pwm_ch1_0_pins[] = {51};
static const unsigned int pwm_ch1_1_pins[] = {73};
static const unsigned int pwm_ch1_2_pins[] = {95};
static const unsigned int pwm_ch2_0_pins[] = {52};
static const unsigned int pwm_ch2_1_pins[] = {74};
static const unsigned int pwm_ch2_2_pins[] = {96};
static const unsigned int pwm_ch3_0_pins[] = {53};
static const unsigned int pwm_ch3_1_pins[] = {75};
static const unsigned int pwm_ch3_2_pins[] = {97};
static const unsigned int pwm_ch4_0_pins[] = {54};
static const unsigned int pwm_ch4_1_pins[] = {67};
static const unsigned int pwm_ch4_2_pins[] = {76};
static const unsigned int pwm_ch4_3_pins[] = {98};
static const unsigned int pwm_ch5_0_pins[] = {68};
static const unsigned int pwm_ch5_1_pins[] = {77};
static const unsigned int pwm_ch5_2_pins[] = {99};
static const unsigned int pwm_ch6_0_pins[] = {69};
static const unsigned int pwm_ch6_1_pins[] = {78};
static const unsigned int pwm_ch6_2_pins[] = {81};
static const unsigned int pwm_ch6_3_pins[] = {100};
static const unsigned int pwm_ch7_0_pins[] = {70};
static const unsigned int pwm_ch7_1_pins[] = {82};
static const unsigned int pwm_ch7_2_pins[] = {101};
static const struct mt7622_group_desc pwm_groups[] = {
        { "pwm_ch1_0", pwm_ch1_0_pins, nitems(pwm_ch1_0_pins) },
        { "pwm_ch1_1", pwm_ch1_1_pins, nitems(pwm_ch1_1_pins) },
        { "pwm_ch1_2", pwm_ch1_2_pins, nitems(pwm_ch1_2_pins) },
        { "pwm_ch2_0", pwm_ch2_0_pins, nitems(pwm_ch2_0_pins) },
        { "pwm_ch2_1", pwm_ch2_1_pins, nitems(pwm_ch2_1_pins) },
        { "pwm_ch2_2", pwm_ch2_2_pins, nitems(pwm_ch2_2_pins) },
        { "pwm_ch3_0", pwm_ch3_0_pins, nitems(pwm_ch3_0_pins) },
        { "pwm_ch3_1", pwm_ch3_1_pins, nitems(pwm_ch3_1_pins) },
        { "pwm_ch3_2", pwm_ch3_2_pins, nitems(pwm_ch3_2_pins) },
        { "pwm_ch4_0", pwm_ch4_0_pins, nitems(pwm_ch4_0_pins) },
        { "pwm_ch4_1", pwm_ch4_1_pins, nitems(pwm_ch4_1_pins) },
        { "pwm_ch4_2", pwm_ch4_2_pins, nitems(pwm_ch4_2_pins) },
        { "pwm_ch4_3", pwm_ch4_3_pins, nitems(pwm_ch4_3_pins) },
        { "pwm_ch5_0", pwm_ch5_0_pins, nitems(pwm_ch5_0_pins) },
        { "pwm_ch5_1", pwm_ch5_1_pins, nitems(pwm_ch5_1_pins) },
        { "pwm_ch5_2", pwm_ch5_2_pins, nitems(pwm_ch5_2_pins) },
        { "pwm_ch6_0", pwm_ch6_0_pins, nitems(pwm_ch6_0_pins) },
        { "pwm_ch6_1", pwm_ch6_1_pins, nitems(pwm_ch6_1_pins) },
        { "pwm_ch6_2", pwm_ch6_2_pins, nitems(pwm_ch6_2_pins) },
        { "pwm_ch6_3", pwm_ch6_3_pins, nitems(pwm_ch6_3_pins) },
        { "pwm_ch7_0", pwm_ch7_0_pins, nitems(pwm_ch7_0_pins) },
        { "pwm_ch7_1", pwm_ch7_1_pins, nitems(pwm_ch7_1_pins) },
        { "pwm_ch7_2", pwm_ch7_2_pins, nitems(pwm_ch7_2_pins) },
};

/* sd groups */
static const unsigned int sd_0_pins[] = {16, 17, 18, 19, 20, 21};
static const unsigned int sd_1_pins[] = {25, 26, 27, 28, 29, 30};
static const struct mt7622_group_desc sd_groups[] = {
        { "sd_0", sd_0_pins, nitems(sd_0_pins) },
        { "sd_1", sd_1_pins, nitems(sd_1_pins) },
};

/* spi groups */
static const unsigned int spic0_0_pins[] = {63, 64, 65, 66};
static const unsigned int spic0_1_pins[] = {79, 80, 81, 82};
static const unsigned int spic1_0_pins[] = {67, 68, 69, 70};
static const unsigned int spic1_1_pins[] = {73, 74, 75, 76};
static const unsigned int spic2_0_wp_hold_pins[] = {8, 9};
static const unsigned int spic2_0_pins[] = {10, 11, 12, 13};
static const struct mt7622_group_desc spi_groups[] = {
        { "spic0_0", spic0_0_pins, nitems(spic0_0_pins) },
        { "spic0_1", spic0_1_pins, nitems(spic0_1_pins) },
        { "spic1_0", spic1_0_pins, nitems(spic1_0_pins) },
        { "spic1_1", spic1_1_pins, nitems(spic1_1_pins) },
        { "spic2_0_wp_hold", spic2_0_wp_hold_pins, nitems(spic2_0_wp_hold_pins) },
        { "spic2_0", spic2_0_pins, nitems(spic2_0_pins) },
};

/* tdm groups */
static const unsigned int tdm_0_out_mclk_bclk_ws_pins[] = {8, 9, 10};
static const unsigned int tdm_0_in_mclk_bclk_ws_pins[] = {11, 12, 13};
static const unsigned int tdm_0_out_data_pins[] = {20};
static const unsigned int tdm_0_in_data_pins[] = {21};
static const unsigned int tdm_1_out_mclk_bclk_ws_pins[] = {57, 58, 59};
static const unsigned int tdm_1_in_mclk_bclk_ws_pins[] = {60, 61, 62};
static const unsigned int tdm_1_out_data_pins[] = {55};
static const unsigned int tdm_1_in_data_pins[] = {56};
static const struct mt7622_group_desc tdm_groups[] = {
        { "tdm_0_out_mclk_bclk_ws", tdm_0_out_mclk_bclk_ws_pins, nitems(tdm_0_out_mclk_bclk_ws_pins) },
        { "tdm_0_in_mclk_bclk_ws", tdm_0_in_mclk_bclk_ws_pins, nitems(tdm_0_in_mclk_bclk_ws_pins) },
        { "tdm_0_out_data", tdm_0_out_data_pins, nitems(tdm_0_out_data_pins) },
        { "tdm_0_in_data", tdm_0_in_data_pins, nitems(tdm_0_in_data_pins) },
        { "tdm_1_out_mclk_bclk_ws", tdm_1_out_mclk_bclk_ws_pins, nitems(tdm_1_out_mclk_bclk_ws_pins) },
        { "tdm_1_in_mclk_bclk_ws", tdm_1_in_mclk_bclk_ws_pins, nitems(tdm_1_in_mclk_bclk_ws_pins) },
        { "tdm_1_out_data", tdm_1_out_data_pins, nitems(tdm_1_out_data_pins) },
        { "tdm_1_in_data", tdm_1_in_data_pins, nitems(tdm_1_in_data_pins) },
};

/* uart groups */
static const unsigned int uart0_0_tx_rx_pins[] = {6, 7};
static const unsigned int uart1_0_tx_rx_pins[] = {55, 56};
static const unsigned int uart1_0_rts_cts_pins[] = {57, 58};
static const unsigned int uart1_1_tx_rx_pins[] = {73, 74};
static const unsigned int uart1_1_rts_cts_pins[] = {75, 76};
static const unsigned int uart2_0_tx_rx_pins[] = {3, 4};
static const unsigned int uart2_0_rts_cts_pins[] = {1, 2};
static const unsigned int uart2_1_tx_rx_pins[] = {51, 52};
static const unsigned int uart2_1_rts_cts_pins[] = {53, 54};
static const unsigned int uart2_2_tx_rx_pins[] = {59, 60};
static const unsigned int uart2_2_rts_cts_pins[] = {61, 62};
static const unsigned int uart2_3_tx_rx_pins[] = {95, 96};
static const unsigned int uart3_0_tx_rx_pins[] = {57, 58};
static const unsigned int uart3_1_tx_rx_pins[] = {81, 82};
static const unsigned int uart3_1_rts_cts_pins[] = {79, 80};
static const unsigned int uart4_0_tx_rx_pins[] = {61, 62};
static const unsigned int uart4_1_tx_rx_pins[] = {91, 92};
static const unsigned int uart4_1_rts_cts_pins[] = {93, 94};
static const unsigned int uart4_2_tx_rx_pins[] = {97, 98};
static const unsigned int uart4_2_rts_cts_pins[] = {95, 96};
static const struct mt7622_group_desc uart_groups[] = {
        { "uart0_0_tx_rx", uart0_0_tx_rx_pins, nitems(uart0_0_tx_rx_pins) },
        { "uart1_0_tx_rx", uart1_0_tx_rx_pins, nitems(uart1_0_tx_rx_pins) },
        { "uart1_0_rts_cts", uart1_0_rts_cts_pins, nitems(uart1_0_rts_cts_pins) },
        { "uart1_1_tx_rx", uart1_1_tx_rx_pins, nitems(uart1_1_tx_rx_pins) },
        { "uart1_1_rts_cts", uart1_1_rts_cts_pins, nitems(uart1_1_rts_cts_pins) },
        { "uart2_0_tx_rx", uart2_0_tx_rx_pins, nitems(uart2_0_tx_rx_pins) },
        { "uart2_0_rts_cts", uart2_0_rts_cts_pins, nitems(uart2_0_rts_cts_pins) },
        { "uart2_1_tx_rx", uart2_1_tx_rx_pins, nitems(uart2_1_tx_rx_pins) },
        { "uart2_1_rts_cts", uart2_1_rts_cts_pins, nitems(uart2_1_rts_cts_pins) },
        { "uart2_2_tx_rx", uart2_2_tx_rx_pins, nitems(uart2_2_tx_rx_pins) },
        { "uart2_2_rts_cts", uart2_2_rts_cts_pins, nitems(uart2_2_rts_cts_pins) },
        { "uart2_3_tx_rx", uart2_3_tx_rx_pins, nitems(uart2_3_tx_rx_pins) },
        { "uart3_0_tx_rx", uart3_0_tx_rx_pins, nitems(uart3_0_tx_rx_pins) },
        { "uart3_1_tx_rx", uart3_1_tx_rx_pins, nitems(uart3_1_tx_rx_pins) },
        { "uart3_1_rts_cts", uart3_1_rts_cts_pins, nitems(uart3_1_rts_cts_pins) },
        { "uart4_0_tx_rx", uart4_0_tx_rx_pins, nitems(uart4_0_tx_rx_pins) },
        { "uart4_1_tx_rx", uart4_1_tx_rx_pins, nitems(uart4_1_tx_rx_pins) },
        { "uart4_1_rts_cts", uart4_1_rts_cts_pins, nitems(uart4_1_rts_cts_pins) },
        { "uart4_2_tx_rx", uart4_2_tx_rx_pins, nitems(uart4_2_tx_rx_pins) },
        { "uart4_2_rts_cts", uart4_2_rts_cts_pins, nitems(uart4_2_rts_cts_pins) },
};

/* watchdog groups */
static const unsigned int watchdog_pins[] = {78};
static const struct mt7622_group_desc watchdog_groups[] = {
        {"watchdog", watchdog_pins, nitems(watchdog_pins)},
};

static const struct mt7622_function_desc functions[] = {
        { "emmc", emmc_groups, nitems(emmc_groups) },
        { "eth", eth_groups, nitems(eth_groups) },
        { "i2c", i2c_groups, nitems(i2c_groups) },
        { "ir", ir_groups, nitems(ir_groups) },
        { "led", led_groups, nitems(led_groups)},
        { "flash", flash_groups, nitems(flash_groups) },
        { "pcie", pcie_groups, nitems(pcie_groups) },
        { "pmic", pmic_groups, nitems(pmic_groups) },
        { "pwm", pwm_groups, nitems(pwm_groups) },
        { "sd", sd_groups, nitems(sd_groups) },
        { "spi", spi_groups, nitems(spi_groups) },
        { "tdm", tdm_groups, nitems(tdm_groups) },
        { "uart", uart_groups, nitems(uart_groups) },
        { "watchdog", watchdog_groups, nitems(watchdog_groups) },
};

struct mt7622_pinmux_desc {
    const char *modes[8];
    bus_size_t reg_offset;
    int shift;
};

struct mt7622_pinctrl_softc {
    device_t dev;
    struct resource *mem_res;
    int mem_rid;
    const struct mt7622_pinmux_desc *pixmux;
    const struct mt7622_function_desc *functions;
    size_t nfunctions;
};

static const struct mt7622_pinmux_desc pinmux[] = {
    // number of pin , mode 0..7, register, shift
    [23] = {{ "MDIO", "GPIO", NULL, NULL, NULL, NULL, NULL, NULL}, 0x300, 24},
    [24] = {{ NULL, "GPIO", NULL, NULL, NULL, NULL, NULL, NULL}, 0x300, 24},
    [37] = {{ "Parallel NAND Flash", "GPIO", "EMMC", NULL, NULL, NULL, NULL, NULL}, 0x300, 20},
    [50] = {{ NULL, "GPIO", NULL, NULL, NULL, NULL, NULL, NULL}, 0x300, 20},
    [71] = {{ "PMIC I2C", "GPIO", NULL, NULL, NULL, NULL, NULL, NULL}, 0x300, 16},
    [72] = {{ NULL, "GPIO", NULL, NULL, NULL, NULL, NULL, NULL}, 0x300, 16},
    [25] = {{ "RGMII", "GPIO", "SDXC", NULL, NULL, NULL, NULL, NULL}, 0x300, 12},
    [36] = {{ NULL, "GPIO", NULL, NULL, NULL, NULL, NULL, NULL}, 0x300, 12},
    [10] = {{ "SPI NOR Flash", "GPIO", "SPI NAND Flash", NULL, NULL, NULL, NULL, NULL}, 0x300, 8},
    [13] = {{ NULL, "GPIO", NULL, NULL, NULL, NULL, NULL, NULL}, 0x300, 8},
    [6] = {{ "UART0", "GPIO", NULL, NULL, NULL, NULL, NULL, NULL}, 0x300, 4},
    [7] = {{ NULL, "GPIO", NULL, NULL, NULL, NULL, NULL, NULL}, 0x300, 4},

    [21] = {{ "I2S4_OUT", "GPIO21", "SD_CMD", NULL, NULL, "ANTSEL", "BT_SPXT_C0", "DBG_UTIF"}, 0x310, 28},
    [20] = {{ "I2S3_OUT", "GPIO20", "SD_CLK", NULL, NULL, "ANTSEL", "BT_SPXT_C1", "DBG_UTIF"}, 0x310, 24},
    [19] = {{ "I2S2_OUT", "GPIO19", "SD_D0", NULL, NULL,"ANTSEL", "BT_IPATH_EN", "DBG_UTIF"}, 0x310, 20},
    [18] = {{ "I2S4_IN", "GPIO18", "SD_D1", NULL, NULL, "ANTSEL", "BT_ERX_EN", "DBG_UTIF"}, 0x310, 16},
    [76] = {{ "SPIC1_CS", "GPIO76", "UART CTS1", "I2C2_SDA", "PWM_CH4", "ANTSEL", NULL, "DBG_UTIF"}, 0x310, 12},
    [75] = {{ "SPIC1_MISO", "GPIO75", "UART RTS1", "I2C2_SCL", "PWM_CH3", "ANTSEL", NULL, "DBG_UTIF"}, 0x310, 8},
    [74] = {{ "SPIC1_MOSI", "GPIO74", "UART RDX1", "I2C2_SDA", "PWM_CH4", "ANTSEL", NULL, "DBG_UTIF"}, 0x310, 4},
    [73] = {{ "SPIC1_CLK", "GPIO73", "UART TXD1", "I2C1_SCL", "PWM_CH1", "ANTSEL", NULL, "DBG_UTIF"}, 0x310, 0},

    [77] = {{ "GPIO_D/GPIO77", "GPIO77", NULL, NULL, "PWM_CH5", "ANTSEL", NULL, "DBG_UTIF"}, 0x320, 28},
    [17] = {{ "I2S3_IN", "GPIO17", "SD_D2", NULL, "IR_R", "ANTSEL", "BT_ELNA_EN", "DBG_UTIF"}, 0x320, 24},
    [16] = {{ "I2S2_IN", "GPIO17", "SD_D2", NULL, "IR_R", "ANTSEL", "BT_ELNA_EN", "DBG_UTIF"}, 0x320, 20},
    [0] = {{ "GPIO_A/GPIO0", "GPIO0", NULL, NULL, NULL, NULL, NULL, NULL}, 0x320, 16},
    [78] = {{ "WATCHDOG", "GPIO78", NULL, NULL, "PWM_CH6", NULL, NULL, "DBG_UTIF"}, 0x320, 12},
    [35] = {{ "GPIO_A/GPIO0", "GPIO35", "PCIE0_PAD_CLKREQ", "PCIE1_PAD_CLKREQ", NULL, "ANTSEL", NULL, NULL}, 0x320, 8},
    [34] = {{ "GPIO_A/GPIO0", "GPIO34", "PCIE0_PAD_WAKE", "PCIE1_PAD_WAKE", NULL, "ANTSEL", NULL, "EXT_BGCK"}, 0x320, 4},
    [5] = {{ "I2S_MCLK", "GPIO5", NULL, NULL, NULL, NULL, NULL, "DBG_UTIF"}, 0x320, 0},

    [57] = {{ "I2C2_SCL", "GPIO57", "UART_RTS1", "TDM_OUT_MCLK", NULL, NULL, NULL, NULL}, 0x330, 28},
    [56] = {{ "I2C1_SDA", "GPIO56", "UART_RXD1", "TDM_IN_DATA", NULL, NULL, NULL, NULL}, 0x330, 24},
    [55] = {{ "I2C1_SCL", "GPIO55", "UART_TXD1", "TDM_OUT_DATA", NULL, NULL, NULL, NULL}, 0x330, 20},
    [54] = {{ "UART_CTS2", "GPIO54", NULL, "PWM_CH4", NULL, NULL, NULL, NULL}, 0x330, 16},
    [53] = {{ "UART_RTS2", "GPIO53", NULL, "PWM_CH3", NULL, NULL, NULL, NULL}, 0x330, 12},
    [52] = {{ "UART_RXD2", "GPIO52", NULL, "PWM_CH2", NULL, NULL, NULL, NULL}, 0x330, 8},
    [51] = {{ "UART_TXD2", "GPIO51", NULL, "PWM_CH1", NULL, NULL, NULL, NULL}, 0x330, 4},
    [84] = {{ "PCIE1_PAD_PERST", "GPIO84", NULL, NULL, NULL, NULL, NULL, NULL}, 0x330, 0},

    [65] = {{ "ESW_RXD", "GPIO65", "G1_RXD", NULL, "SPIC0_MISO", NULL, NULL, NULL}, 0x340, 28},
    [64] = {{ "ESW_TXC", "GPIO64", "G1_TXC", NULL, "SPIC0_MOSI", NULL, NULL, NULL}, 0x340, 24},
    [63] = {{ "ESW_TXEN", "GPIO63", "G1_TXEN", NULL, "SPIC0_CLK", NULL, NULL, NULL}, 0x340, 20},
    [62] = {{ "ESW_TXD", "GPIO62", "G1_TXD", "TDM_IN_WS", "UART_CTS2_N", "UART_RXD4", NULL, NULL}, 0x340, 16},
    [61] = {{ "ESW_TXD", "GPIO61", "G1_TXD", "TDM_IN_BCLK", "UART_RTS2_N", "UART_TXD4", NULL, NULL}, 0x340, 12},
    [60] = {{ "ESW_TXD", "GPIO60", "G1_TXD", "TDM_IN_MCLK", "UART_RXD2", "IR_R", NULL, NULL}, 0x340, 8},
    [59] = {{ "ESW_TXD", "GPIO59", "G1_TXD", "TMD_OUT_WS", "UART_TXD2", "IR_T", NULL, NULL}, 0x340, 4},
    [58] = {{ "I2C2_SDA", "GPIO58", "UART_CTS1", "TDM_OUT_BCLK", NULL, NULL, NULL, NULL}, 0x340, 0},

    [83] = {{ "PCIE0_PAD_PERST", "GPIO83", NULL, NULL, NULL, NULL, NULL, NULL}, 0x350, 28},
    [9] = {{ "SPI_HOLD", "GPIO9", "SNFI_HOLD", "TDM_OUT_BCLK", NULL, NULL, "FPC_DL_STS", NULL}, 0x350, 24},
    [8] = {{ "SPI_WP", "GPIO8", "SNFI_WP", "TDM_OUT_MCLK", NULL, NULL, "FPC_DAT_STS", NULL}, 0x350, 20},
    [70] = {{ "ESW_RXC", "GPIO70", "G1_RXCV", "SPIC1_CS", NULL, NULL, NULL, NULL}, 0x350, 16},
    [69] = {{ "ESW_RXDV", "GPIO69", "G1_RXDV", "PWM_CH6", "SPIC1_MISO", NULL, NULL, NULL}, 0x350, 12},
    [68] = {{ "ESW_RXD", "GPIO68", "G1_RXD", "PWM_CH5", "SPIC1_MOSI", NULL, NULL, NULL}, 0x350, 8},
    [67] = {{ "ESW_RXD", "GPIO67", "G1_RXD", "PWM_CH4", "SPIC1_CLK", NULL, NULL, NULL}, 0x350, 4},
    [66] = {{ "ESW_RXD", "GPIO66", "G1_RXD", NULL, "SPIC0_CS", NULL, NULL, NULL}, 0x350, 0},

    [90] = {{ "I2C2_SDA", "GPIO90", NULL, NULL, NULL, NULL, NULL, NULL}, 0x360, 24},
    [89] = {{ "I2C2_SCL", "GPIO89", NULL, NULL, NULL, NULL, NULL, NULL}, 0x360, 20},
    [88] = {{ "I2C1_SDA", "GPIO88", NULL, NULL, NULL, NULL, NULL, NULL}, 0x360, 16},
    [87] = {{ "I2C1_SCL", "GPIO87", NULL, NULL, NULL, NULL, NULL, NULL}, 0x360, 12},
    [86] = {{ "EPHY_LED0_N", "GPIO86", NULL, NULL, "CPUM_HW_SEL", NULL, "FPC_CTL", "JTRST_N"}, 0x360, 8},
    [85] = {{ "WLED_N", "GPIO85", NULL, NULL, NULL, NULL, NULL, NULL}, 0x360, 4},
    [102] = {{ "GPIO_E/GPIO102", "GPIO102", NULL, NULL, NULL, NULL, "ANTSEL", "FPC_DATA"}, 0x360, 0},

    [97] = {{ "PWM_CH3", "GPIO97", "UART_TXD4", NULL, "AICE_TCKC", "ANTSEL", "FPC_DATA[", "W_JTCLK"}, 0x380, 28},
    [96] = {{ "PWM_CH2", "GPIO96", "UART_CTS4", "UART_RXD2", "CPUM_CK_XI", "ANTSEL", "FPC_DATA", "W_DBGACK"}, 0x380, 24},
    [95] = {{ "I2C1_SDA", "GPIO88", NULL, NULL, NULL, NULL, NULL, NULL}, 0x380, 20},
    [22] = {{ "GPIO_B/GPIO22", "GPIO22", NULL, "TSF_INTR", NULL, NULL, "ANTSEL", "DBG_UTIF"}, 0x380, 16},

    [94] = {{ "UART_CTS4", "GPIO94", "EPHY_LED4_N", "DFD_TMS", "CPUM", "ANTSEL", "FPC_CTL", "JTMS"}, 0x390, 28},
    [93] = {{ "UART_RTS4", "GPIO93", "EPHY_LED3_N", "DFD_TCK", "CPUM", "ANTSEL", "FPC_CTL", "JTCLK"}, 0x390, 24},
    [92] = {{ "UART_RXD4", "GPIO92", "EPHY_LED2_N", "DFD_TDO", "CPUM", "ANTSEL", "FPC_CTL", "JTDO"}, 0x390, 20},
    [91] = {{ "UART_TXD4", "GPIO91", "EPHY_LED1_N", "DFD_TDI", "CPUM_2B_SEL", "ANTSEL", "FPC_CK_XI", "JTDI"}, 0x390, 16},
    [101] = {{ "PWM_CH7", "GPIO101", NULL, NULL, NULL, "ANTSEL", "FPC_DATA", "DBG_UART_TXD"}, 0x390, 12},
    [100] = {{ "PWM_CH6", "GPIO100", NULL, "IR_R", NULL, "ANTSEL", "FPC_DATA", "W_JTRST_N"}, 0x390, 8},
    [99] = {{ "PWM_CH5", "GPIO99", NULL, "IR_T", "AICE_TMSC", "ANTSEL", "FPC_DATA", "W_JTMS"}, 0x390, 4},
    [98] = {{ "PWM_CH4", "GPIO98", "UART_RXD4", NULL, NULL, "ANTSEL", "FPC_DATA", "W_JTDI"}, 0x390, 0},

    [4] = {{ "I2S_WS_OUT", "GPIO4", "UART_RXD2", "i2S_WS_IN", NULL, NULL, NULL, NULL}, 0x3A0, 28},
    [3] = {{ "I2S_BCLK_OUT", "GPIO3", "UART_TXD2", "i2S_BCLK_IN", NULL, NULL, NULL, NULL}, 0x3A0, 24},
    [2] = {{ "I2S1_OUT", "GPIO2", "UART_CTS2_N", NULL, NULL, NULL, NULL, NULL}, 0x3A0, 20},
    [1] = {{ "I2S1_IN", "GPIO1", "UART_RTS2_N", NULL, NULL, NULL, NULL, NULL}, 0x3A0, 16},
    [82] = {{ "UART_RXD3", "GPIO82", "SPDIF_R", "SPIC0_MOSI", "PWM_CH7", "ANTSEL", NULL, "DBG_UTIF"}, 0x3A0, 12},
    [81] = {{ "UART_TXD3", "GPIO81", "SPDIF_T", "SPIC0_CLK", "PWM_CH6", "ANTSEL", NULL, "DBG_UTIF"}, 0x3A0, 8},
    [80] = {{ "UART_CTS3", "GPIO80", NULL, "SPIC0_CS", "CPIE0_PAD_CLKREQ", "ANTSEL", NULL, "DBG_UTIF"}, 0x3A0, 4},
    [79] = {{ "UART_RTS3", "GPIO79", NULL, "SPIC0_MISO", "PCIE0_PAD_WAKE", "ANTSEL", NULL, "DBG_UTIF"}, 0x3A0, 0}
};

static const struct
mt7622_function_desc *find_function(struct mt7622_pinctrl_softc *sc, const char *name) {
    if (!sc || !sc->functions || !name) {
        return NULL;
    }

    for (size_t i = 0; i < sc->nfunctions; i++) {
        if (!sc->functions[i].name) {
            continue; // skip empty slots
        }

        if (strcmp(sc->functions[i].name, name) == 0) {
            return &sc->functions[i];
        }
    }
    return NULL;
}

static const struct
mt7622_group_desc *find_group_in_function(const struct mt7622_function_desc *function, const char *group) {
    for (size_t i = 0; i < function->ngroups; i++) {
        if (strcmp(function->groups[i].name, group) == 0) {
            return &function->groups[i];
        }
    }
    return NULL;
}

static int
find_mode_index(const struct mt7622_pinmux_desc *desc, const char *mode) {
    for (int i = 0; i < 8; i++) {
        if(desc->modes[i] != NULL) {
            if (strncasecmp(desc->modes[i], mode, 32) == 0) {
                return i;
            }
        }
    }
    return -1;
}

static int
mt7622_pinctrl_process_node(struct mt7622_pinctrl_softc *sc, phandle_t node) {
    char name[64];
    const char **groups = NULL;
    char *function = NULL;
    int num_groups;

    OF_getprop(node, "name", name, sizeof(name));
    if (strncmp(name, "mux", 3) == 0) {
        num_groups = ofw_bus_string_list_to_array(node, "groups", &groups);

        if (num_groups <= 0)
            return (ENOENT);

        if (OF_getprop_alloc_multi(node, "function", sizeof(*function),
                                   (void **)&function) == -1) {
            //ret = ENOENT;
            goto out;
        }

        const struct mt7622_function_desc *func = find_function(sc, function);
        if (!func) {
            device_printf(sc->dev, "Unknown function: %s\n", function);
            OF_prop_free(groups);
            return (EINVAL);
        } else {
            device_printf(sc->dev, "found function: %s\n", func->name ? func->name : "<NULL>");
        }

        for (int i = 0; i < num_groups; i++) {
            const struct mt7622_group_desc *grp = find_group_in_function(func, groups[i]);
            if (!grp) {
                device_printf(sc->dev, "Unknown group: %s for function %s\n", groups[i], function);
            }
            else
            {
                device_printf(sc->dev, "group: %s for function %s\n", grp->name, function);
            }

            for (size_t i = 0; i < grp->npins; i++) {
                int pin = grp->pins[i];
                const struct mt7622_pinmux_desc *desc = &pinmux[pin];
                int mode = find_mode_index(desc, func->name);
                if (mode < 0) {
                    device_printf(sc->dev, "Pin %d does not support mode %d\n", pin, mode);
                }
                else {
                    device_printf(sc->dev, "Pin %d does support mode %d\n", pin, mode);

                    uint32_t mask = 0xF;
                    uint32_t val = bus_read_4(sc->mem_res, desc->reg_offset);
                    val &= ~(mask << desc->shift);
                    val |= ((mode & mask) << desc->shift);
                    bus_write_4(sc->mem_res, desc->reg_offset, val);

                    device_printf(sc->dev,
                                  "Pinmux: pin=%d, reg=0x%08lx, shift=%d, mode=%d\n",
                                  pin, desc->reg_offset, desc->shift, mode);
                }
            }
        }


    }
    out:
        OF_prop_free(groups);
        OF_prop_free(function);
        return (0);
}

static int
mt7622_pinctrl_configure(device_t dev, phandle_t cfgxref) {
    struct mt7622_pinctrl_softc *sc;
    phandle_t node, cfgnode;
    sc = device_get_softc(dev);
    cfgnode = OF_node_from_xref(cfgxref);
    if (cfgnode <= 0)
        return (ENXIO);

    for (node = OF_child(cfgnode); node != 0; node = OF_peer(node)) {
        mt7622_pinctrl_process_node(sc, node);
    }

    return 0;
}

static int
mt7622_pinctrl_probe(device_t dev) {
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
        return (ENXIO);

    device_set_desc(dev, "Mediatek 7622 pinctrl configuration");
    return (BUS_PROBE_DEFAULT);
}

static int
mt7622_pinctrl_attach(device_t dev) {
    struct mt7622_pinctrl_softc *sc = device_get_softc(dev);
    //phandle_t node;
    sc->dev = dev;
    //node = ofw_bus_get_node(dev);

    /* Map memory resource */
    sc->mem_rid = 0;
    sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid, RF_ACTIVE);
    if (sc->mem_res == NULL) {
        device_printf(dev, "Could not map memory resource\n");
        return (ENXIO);
    }

    sc->pixmux = pinmux;
    sc->functions = functions;
    sc->nfunctions = nitems(functions);

    fdt_pinctrl_register(dev, NULL);
    //simplebus_init(dev, node);
    //bus_identify_children(dev);
    fdt_pinctrl_configure_tree(dev);
    //bus_attach_children(dev);
     return (0);
}

static int mt7622_pinctrl_detach(device_t dev) {
    struct mt7622_pinctrl_softc *sc = device_get_softc(dev);
    if (sc->mem_res) {
        bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
        sc->mem_res = NULL;
    }

    return (0);
}

static device_method_t mt7622_pinctrl_methods[] = {
    DEVMETHOD(device_probe, mt7622_pinctrl_probe),
    DEVMETHOD(device_attach, mt7622_pinctrl_attach),
    DEVMETHOD(device_detach, mt7622_pinctrl_detach),
    DEVMETHOD(fdt_pinctrl_configure, mt7622_pinctrl_configure),
    DEVMETHOD_END
};

static DEFINE_CLASS_0(mt7622_pinctrl, mt7622_pinctrl_driver, mt7622_pinctrl_methods,
                      sizeof(struct mt7622_pinctrl_softc));
EARLY_DRIVER_MODULE(mt7622_pinctrl, simplebus, mt7622_pinctrl_driver, NULL, NULL,
                    71);
