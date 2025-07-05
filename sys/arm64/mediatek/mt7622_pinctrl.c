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
#include <dev/gpio/gpiobusvar.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/openfirm.h>

static struct ofw_compat_data compat_data[] =
    {{"mediatek,mt7622-pinctrl", 1}, {NULL, 0}};

/*
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

struct pinctrl_pin_desc {
    unsigned int number;
    const char *name;
};

struct pinctrl_pin_group {
    const char *name;
    const unsigned int *pins;
    unsigned int npins;
    const void *data;
};

struct pinctrl_function {
    const char *name;
    const char *const *groups;
    unsigned int ngroups;
};

struct mt7622_pinctrl_fdt_cfg {
    uint32_t pin;
    uint32_t func;
    uint32_t config;
};

struct mt7622_pinctrl_irqsrc {
    struct intr_irqsrc isrc;
    int irq;
    int type;
};

struct mt7622_pinctrl_softc {
    device_t dev;
    device_t sc_dev;
    device_t sc_busdev;
    struct mtx sc_mtx;
    int sc_ngpios;
    struct mt7622_pinctrl_irqsrc *sc_irqs;

    struct resource *mem_res;
    int mem_rid;

    const struct pinctrl_pin_desc *pins;
    unsigned int npins;

    const struct pinctrl_pin_group *groups;
    unsigned int ngroups;

    const struct pinctrl_function *functions;
    unsigned int nfunctions;
};*/
/*
static const struct pinctrl_pin_desc mt7622_pins[] = {
{.number = 0, .name = "GPIO_A"},       {.number = 1, .name = "I2S1_IN"},
{.number = 2, .name = "I2S1_OUT"},     {.number = 3, .name = "I2S_BCLK"},
{.number = 4, .name = "I2S_WS"},       {.number = 5, .name = "I2S_MCLK"},
{.number = 6, .name = "TXD0"},         {.number = 7, .name = "RXD0"},
{.number = 8, .name = "SPI_WP"},       {.number = 9, .name = "SPI_HOLD"},
{.number = 10, .name = "SPI_CLK"},     {.number = 11, .name = "SPI_MOSI"},
{.number = 12, .name = "SPI_MISO"},    {.number = 13, .name = "SPI_CS"},
{.number = 14, .name = "I2C_SDA"},     {.number = 15, .name = "I2C_SCL"},
{.number = 16, .name = "I2S2_IN"},     {.number = 17, .name = "I2S3_IN"},
{.number = 18, .name = "I2S4_IN"},     {.number = 19, .name = "I2S2_OUT"},
{.number = 20, .name = "I2S3_OUT"},    {.number = 21, .name = "I2S4_OUT"},
{.number = 22, .name = "GPIO_B"},      {.number = 23, .name = "MDC"},
{.number = 24, .name = "MDIO"},        {.number = 25, .name = "G2_TXD0"},
{.number = 26, .name = "G2_TXD1"},     {.number = 27, .name = "G2_TXD2"},
{.number = 28, .name = "G2_TXD3"},     {.number = 29, .name = "G2_TXEN"},
{.number = 30, .name = "G2_TXC"},      {.number = 31, .name = "G2_RXD0"},
{.number = 32, .name = "G2_RXD1"},     {.number = 33, .name = "G2_RXD2"},
{.number = 34, .name = "G2_RXD3"},     {.number = 35, .name = "G2_RXDV"},
{.number = 36, .name = "G2_RXC"},      {.number = 37, .name = "NCEB"},
{.number = 38, .name = "NWEB"},        {.number = 39, .name = "NREB"},
{.number = 40, .name = "NDL4"},        {.number = 41, .name = "NDL5"},
{.number = 42, .name = "NDL6"},        {.number = 43, .name = "NDL7"},
{.number = 44, .name = "NRB"},         {.number = 45, .name = "NCLE"},
{.number = 46, .name = "NALE"},        {.number = 47, .name = "NDL0"},
{.number = 48, .name = "NDL1"},        {.number = 49, .name = "NDL2"},
{.number = 50, .name = "NDL3"},        {.number = 51, .name = "MDI_TP_P0"},
{.number = 52, .name = "MDI_TN_P0"},   {.number = 53, .name = "MDI_RP_P0"},
{.number = 54, .name = "MDI_RN_P0"},   {.number = 55, .name = "MDI_TP_P1"},
{.number = 56, .name = "MDI_TN_P1"},   {.number = 57, .name = "MDI_RP_P1"},
{.number = 58, .name = "MDI_RN_P1"},   {.number = 59, .name = "MDI_RP_P2"},
{.number = 60, .name = "MDI_RN_P2"},   {.number = 61, .name = "MDI_TP_P2"},
{.number = 62, .name = "MDI_TN_P2"},   {.number = 63, .name = "MDI_TP_P3"},
{.number = 64, .name = "MDI_TN_P3"},   {.number = 65, .name = "MDI_RP_P3"},
{.number = 66, .name = "MDI_RN_P3"},   {.number = 67, .name = "MDI_RP_P4"},
{.number = 68, .name = "MDI_RN_P4"},   {.number = 69, .name = "MDI_TP_P4"},
{.number = 70, .name = "MDI_TN_P4"},   {.number = 71, .name = "PMIC_SCL"},
{.number = 72, .name = "PMIC_SDA"},    {.number = 73, .name = "SPIC1_CLK"},
{.number = 74, .name = "SPIC1_MOSI"},  {.number = 75, .name = "SPIC1_MISO"},
{.number = 76, .name = "SPIC1_CS"},    {.number = 77, .name = "GPIO_D"},
{.number = 78, .name = "WATCHDOG"},    {.number = 79, .name = "RTS3_N"},
{.number = 80, .name = "CTS3_N"},      {.number = 81, .name = "TXD3"},
{.number = 82, .name = "RXD3"},        {.number = 83, .name = "PERST0_N"},
{.number = 84, .name = "PERST1_N"},    {.number = 85, .name = "WLED_N"},
{.number = 86, .name = "EPHY_LED0_N"}, {.number = 87, .name = "AUXIN0"},
{.number = 88, .name = "AUXIN1"},      {.number = 89, .name = "AUXIN2"},
{.number = 90, .name = "AUXIN3"},      {.number = 91, .name = "TXD4"},
{.number = 92, .name = "RXD4"},        {.number = 93, .name = "RTS4_N"},
{.number = 94, .name = "CTS4_N"},      {.number = 95, .name = "PWM1"},
{.number = 96, .name = "PWM2"},        {.number = 97, .name = "PWM3"},
{.number = 98, .name = "PWM4"},        {.number = 99, .name = "PWM5"},
{.number = 100, .name = "PWM6"},       {.number = 101, .name = "PWM7"},
{.number = 102, .name = "GPIO_E"},
};
#define MT7622_NUM_PINS ARRAY_SIZE(mt7622_pins)

static const unsigned int emmc_pins[] = {40, 41, 42, 43, 44, 45, 47, 48, 49, 50};
static const unsigned int emmc_rst_pins[] = {37};
static const unsigned int esw_pins[] = {51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
static const unsigned int esw_p0_p1_pins[] = {51, 52, 53, 54, 55, 56, 57, 58};
static const unsigned int esw_p2_p3_p4_pins[] = {59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
static const unsigned int rgmii_via_esw_pins[] = {59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
static const unsigned int rgmii_via_gmac1_pins[] = {59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70};
static const unsigned int rgmii_via_gmac2_pins[] = {25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36};
static const unsigned int mdc_mdio_pins[] = {23, 24};

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

static const unsigned int ir_0_tx_pins[] = {16};
static const unsigned int ir_1_tx_pins[] = {59};
static const unsigned int ir_2_tx_pins[] = {99};
static const unsigned int ir_0_rx_pins[] = {17};
static const unsigned int ir_1_rx_pins[] = {60};
static const unsigned int ir_2_rx_pins[] = {100};

static const unsigned int ephy_leds_pins[] = {86, 91, 92, 93, 94};
static const unsigned int ephy0_led_pins[] = {86};
static const unsigned int ephy1_led_pins[] = {91};
static const unsigned int ephy2_led_pins[] = {92};
static const unsigned int ephy3_led_pins[] = {93};
static const unsigned int ephy4_led_pins[] = {94};
static const unsigned int wled_pins[] = {85};

static const unsigned int par_nand_pins[] = {37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50};
static const unsigned int snfi_pins[] = {8, 9, 10, 11, 12, 13};
static const unsigned int spi_nor_pins[] = {8, 9, 10, 11, 12, 13};

static const unsigned int pcie0_0_waken_pins[] = {14};
static const unsigned int pcie0_1_waken_pins[] = {79};
static const unsigned int pcie1_0_waken_pins[] = {14};
static const unsigned int pcie0_0_clkreq_pins[] = {15};
static const unsigned int pcie0_1_clkreq_pins[] = {80};
static const unsigned int pcie1_0_clkreq_pins[] = {15};
static const unsigned int pcie0_pad_perst_pins[] = {83};
static const unsigned int pcie1_pad_perst_pins[] = {84};

static const unsigned int pmic_bus_pins[] = {71, 72};

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

static const unsigned int sd_0_pins[] = {16, 17, 18, 19, 20, 21};
static const unsigned int sd_1_pins[] = {25, 26, 27, 28, 29, 30};

static const unsigned int spic0_0_pins[] = {63, 64, 65, 66};
static const unsigned int spic0_1_pins[] = {79, 80, 81, 82};
static const unsigned int spic1_0_pins[] = {67, 68, 69, 70};
static const unsigned int spic1_1_pins[] = {73, 74, 75, 76};
static const unsigned int spic2_0_wp_hold_pins[] = {8, 9};
static const unsigned int spic2_0_pins[] = {10, 11, 12, 13};

static const unsigned int tdm_0_out_mclk_bclk_ws_pins[] = {8, 9, 10};
static const unsigned int tdm_0_in_mclk_bclk_ws_pins[] = {11, 12, 13};
static const unsigned int tdm_0_out_data_pins[] = {20};
static const unsigned int tdm_0_in_data_pins[] = {21};
static const unsigned int tdm_1_out_mclk_bclk_ws_pins[] = {57, 58, 59};
static const unsigned int tdm_1_in_mclk_bclk_ws_pins[] = {60, 61, 62};
static const unsigned int tdm_1_out_data_pins[] = {55};
static const unsigned int tdm_1_in_data_pins[] = {56};

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

static const unsigned int watchdog_pins[] = {78};

static const struct pinctrl_pin_group mt7622_groups[] = {
{
    .name = "emmc",
    .pins = emmc_pins,
    .npins = ARRAY_SIZE(emmc_pins),
},
{
    .name = "emmc_rst",
    .pins = emmc_rst_pins,
    .npins = ARRAY_SIZE(emmc_rst_pins),
},
{
    .name = "esw",
    .pins = esw_pins,
    .npins = ARRAY_SIZE(esw_pins),
},
{
    .name = "esw_p0_p1",
    .pins = esw_p0_p1_pins,
    .npins = ARRAY_SIZE(esw_p0_p1_pins),
},
{
    .name = "esw_p2_p3_p4",
    .pins = esw_p2_p3_p4_pins,
    .npins = ARRAY_SIZE(esw_p2_p3_p4_pins),
},
{
    .name = "rgmii_via_esw",
    .pins = rgmii_via_esw_pins,
    .npins = ARRAY_SIZE(rgmii_via_esw_pins),
},
{
    .name = "rgmii_via_gmac1",
    .pins = rgmii_via_gmac1_pins,
    .npins = ARRAY_SIZE(rgmii_via_gmac1_pins),
},
{
    .name = "rgmii_via_gmac2",
    .pins = rgmii_via_gmac2_pins,
    .npins = ARRAY_SIZE(rgmii_via_gmac2_pins),
},
{
    .name = "mdc_mdio",
    .pins = i2c0_pins,
    .npins = ARRAY_SIZE(i2c0_pins),
},
{
    .name = "i2c0",
    .pins = mdc_mdio_pins,
    .npins = ARRAY_SIZE(mdc_mdio_pins),
},
{
    .name = "i2c1_0",
    .pins = i2c1_0_pins,
    .npins = ARRAY_SIZE(i2c1_0_pins),
},
{
    .name = "i2c1_1",
    .pins = i2c1_1_pins,
    .npins = ARRAY_SIZE(i2c1_1_pins),
},
{
    .name = "i2c1_2",
    .pins = i2c1_2_pins,
    .npins = ARRAY_SIZE(i2c1_2_pins),
},
{
    .name = "i2c2_0",
    .pins = i2c2_0_pins,
    .npins = ARRAY_SIZE(i2c2_0_pins),
},
{
    .name = "i2c2_1",
    .pins = i2c2_1_pins,
    .npins = ARRAY_SIZE(i2c2_1_pins),
},
{
    .name = "i2c2_2",
    .pins = i2c2_2_pins,
    .npins = ARRAY_SIZE(i2c2_2_pins),
},
{
    .name = "i2s_in_mclk_bclk_ws",
    .pins = i2s_in_mclk_bclk_ws_pins,
    .npins = ARRAY_SIZE(i2s_in_mclk_bclk_ws_pins),
},
{
    .name = "i2s1_in_data",
    .pins = i2s1_in_data_pins,
    .npins = ARRAY_SIZE(i2s1_in_data_pins),
},
{
    .name = "i2s2_in_data",
    .pins = i2s2_in_data_pins,
    .npins = ARRAY_SIZE(i2s2_in_data_pins),
},
{
    .name = "i2s3_in_data",
    .pins = i2s3_in_data_pins,
    .npins = ARRAY_SIZE(i2s3_in_data_pins),
},
{
    .name = "i2s4_in_data",
    .pins = i2s4_in_data_pins,
    .npins = ARRAY_SIZE(i2s4_in_data_pins),
},
{
    .name = "i2s_out_mclk_bclk_ws",
    .pins = i2s_out_mclk_bclk_ws_pins,
    .npins = ARRAY_SIZE(i2s_out_mclk_bclk_ws_pins),
},
{
    .name = "i2s1_out_data",
    .pins = i2s1_out_data_pins,
    .npins = ARRAY_SIZE(i2s1_out_data_pins),
},
{
    .name = "i2s2_out_data",
    .pins = i2s2_out_data_pins,
    .npins = ARRAY_SIZE(i2s2_out_data_pins),
},
{
    .name = "i2s3_out_data",
    .pins = i2s3_out_data_pins,
    .npins = ARRAY_SIZE(i2s3_out_data_pins),
},
{
    .name = "i2s4_out_data",
    .pins = i2s4_out_data_pins,
    .npins = ARRAY_SIZE(i2s4_out_data_pins),
},
{
    .name = "ir_0_tx",
    .pins = ir_0_tx_pins,
    .npins = ARRAY_SIZE(ir_0_tx_pins),
},
{
    .name = "ir_1_tx",
    .pins = ir_1_tx_pins,
    .npins = ARRAY_SIZE(ir_1_tx_pins),
},
{
    .name = "ir_2_tx",
    .pins = ir_2_tx_pins,
    .npins = ARRAY_SIZE(ir_2_tx_pins),
},
{
    .name = "ir_0_rx",
    .pins = ir_0_rx_pins,
    .npins = ARRAY_SIZE(ir_0_rx_pins),
},
{
    .name = "ir_1_rx",
    .pins = ir_1_rx_pins,
    .npins = ARRAY_SIZE(ir_1_rx_pins),
},
{
    .name = "ir_2_rx",
    .pins = ir_2_rx_pins,
    .npins = ARRAY_SIZE(ir_2_rx_pins),
},
{
    .name = "ephy_leds",
    .pins = ephy_leds_pins,
    .npins = ARRAY_SIZE(ephy_leds_pins),
},
{
    .name = "ephy0_led",
    .pins = ephy0_led_pins,
    .npins = ARRAY_SIZE(ephy0_led_pins),
},
{
    .name = "ephy1_led",
    .pins = ephy1_led_pins,
    .npins = ARRAY_SIZE(ephy1_led_pins),
},
{
    .name = "ephy2_led",
    .pins = ephy2_led_pins,
    .npins = ARRAY_SIZE(ephy2_led_pins),
},
{
    .name = "ephy3_led",
    .pins = ephy3_led_pins,
    .npins = ARRAY_SIZE(ephy3_led_pins),
},
{
    .name = "ephy4_led",
    .pins = ephy4_led_pins,
    .npins = ARRAY_SIZE(ephy4_led_pins),
},
{
    .name = "wled",
    .pins = wled_pins,
    .npins = ARRAY_SIZE(wled_pins),
},
{
    .name = "par_nand",
    .pins = par_nand_pins,
    .npins = ARRAY_SIZE(par_nand_pins),
},
{
    .name = "snfi",
    .pins = snfi_pins,
    .npins = ARRAY_SIZE(snfi_pins),
},
{
    .name = "spi_nor",
    .pins = spi_nor_pins,
    .npins = ARRAY_SIZE(spi_nor_pins),
},
{
    .name = "pcie0_0_waken",
    .pins = pcie0_0_waken_pins,
    .npins = ARRAY_SIZE(pcie0_0_waken_pins),
},
{
    .name = "pcie0_1_waken",
    .pins = pcie0_1_waken_pins,
    .npins = ARRAY_SIZE(pcie0_1_waken_pins),
},
{
    .name = "pcie1_0_waken",
    .pins = pcie1_0_waken_pins,
    .npins = ARRAY_SIZE(pcie1_0_waken_pins),
},
{
    .name = "pcie0_0_clkreq",
    .pins = pcie0_0_clkreq_pins,
    .npins = ARRAY_SIZE(pcie0_0_clkreq_pins),
},
{
    .name = "pcie0_1_clkreq",
    .pins = pcie0_1_clkreq_pins,
    .npins = ARRAY_SIZE(pcie0_1_clkreq_pins),
},
{
    .name = "pcie1_0_clkreq",
    .pins = pcie1_0_clkreq_pins,
    .npins = ARRAY_SIZE(pcie1_0_clkreq_pins),
},
{
    .name = "pcie0_pad_perst",
    .pins = pcie0_pad_perst_pins,
    .npins = ARRAY_SIZE(pcie0_pad_perst_pins),
},
{
    .name = "pcie1_pad_perst",
    .pins = pcie1_pad_perst_pins,
    .npins = ARRAY_SIZE(pcie1_pad_perst_pins),
},
{
    .name = "pmic_bus",
    .pins = pmic_bus_pins,
    .npins = ARRAY_SIZE(pmic_bus_pins),
},
{
    .name = "pwm_ch1_0",
    .pins = pwm_ch1_0_pins,
    .npins = ARRAY_SIZE(pwm_ch1_0_pins),
},
{
    .name = "pwm_ch1_1",
    .pins = pwm_ch1_1_pins,
    .npins = ARRAY_SIZE(pwm_ch1_1_pins),
},
{
    .name = "pwm_ch1_2",
    .pins = pwm_ch1_2_pins,
    .npins = ARRAY_SIZE(pwm_ch1_2_pins),
},
{
    .name = "pwm_ch2_0",
    .pins = pwm_ch2_0_pins,
    .npins = ARRAY_SIZE(pwm_ch2_0_pins),
},
{
    .name = "pwm_ch2_1",
    .pins = pwm_ch2_1_pins,
    .npins = ARRAY_SIZE(pwm_ch2_1_pins),
},
{
    .name = "pwm_ch2_2",
    .pins = pwm_ch2_2_pins,
    .npins = ARRAY_SIZE(pwm_ch2_2_pins),
},
{
    .name = "pwm_ch3_0",
    .pins = pwm_ch3_0_pins,
    .npins = ARRAY_SIZE(pwm_ch3_0_pins),
},
{
    .name = "pwm_ch3_1",
    .pins = pwm_ch3_1_pins,
    .npins = ARRAY_SIZE(pwm_ch3_1_pins),
},
{
    .name = "pwm_ch3_2",
    .pins = pwm_ch3_2_pins,
    .npins = ARRAY_SIZE(pwm_ch3_2_pins),
},
{
    .name = "pwm_ch4_0",
    .pins = pwm_ch4_0_pins,
    .npins = ARRAY_SIZE(pwm_ch4_0_pins),
},
{
    .name = "pwm_ch4_1",
    .pins = pwm_ch4_1_pins,
    .npins = ARRAY_SIZE(pwm_ch4_1_pins),
},
{
    .name = "pwm_ch4_2",
    .pins = pwm_ch4_2_pins,
    .npins = ARRAY_SIZE(pwm_ch4_2_pins),
},
{
    .name = "pwm_ch4_3",
    .pins = pwm_ch4_3_pins,
    .npins = ARRAY_SIZE(pwm_ch4_3_pins),
},
{
    .name = "pwm_ch5_0",
    .pins = pwm_ch5_0_pins,
    .npins = ARRAY_SIZE(pwm_ch5_0_pins),
},
{
    .name = "pwm_ch5_1",
    .pins = pwm_ch5_1_pins,
    .npins = ARRAY_SIZE(pwm_ch5_1_pins),
},
{
    .name = "pwm_ch5_2",
    .pins = pwm_ch5_2_pins,
    .npins = ARRAY_SIZE(pwm_ch5_2_pins),
},
{
    .name = "pwm_ch6_0",
    .pins = pwm_ch6_0_pins,
    .npins = ARRAY_SIZE(pwm_ch6_0_pins),
},
{
    .name = "pwm_ch6_1",
    .pins = pwm_ch6_1_pins,
    .npins = ARRAY_SIZE(pwm_ch6_1_pins),
},
{
    .name = "pwm_ch6_2",
    .pins = pwm_ch6_2_pins,
    .npins = ARRAY_SIZE(pwm_ch6_2_pins),
},
{
    .name = "pwm_ch6_3",
    .pins = pwm_ch6_3_pins,
    .npins = ARRAY_SIZE(pwm_ch6_3_pins),
},
{
    .name = "pwm_ch7_0",
    .pins = pwm_ch7_0_pins,
    .npins = ARRAY_SIZE(pwm_ch7_0_pins),
},
{
    .name = "pwm_ch7_1",
    .pins = pwm_ch7_1_pins,
    .npins = ARRAY_SIZE(pwm_ch7_1_pins),
},
{
    .name = "pwm_ch7_2",
    .pins = pwm_ch7_2_pins,
    .npins = ARRAY_SIZE(pwm_ch7_2_pins),
},
{
    .name = "sd_0",
    .pins = sd_0_pins,
    .npins = ARRAY_SIZE(sd_0_pins),
},
{
    .name = "sd_1",
    .pins = sd_1_pins,
    .npins = ARRAY_SIZE(sd_1_pins),
},
{
    .name = "spic0_0",
    .pins = spic0_0_pins,
    .npins = ARRAY_SIZE(spic0_0_pins),
},
{
    .name = "spic0_1",
    .pins = spic0_1_pins,
    .npins = ARRAY_SIZE(spic0_1_pins),
},
{
    .name = "spic1_0",
    .pins = spic1_0_pins,
    .npins = ARRAY_SIZE(spic1_0_pins),
},
{
    .name = "spic1_1",
    .pins = spic1_1_pins,
    .npins = ARRAY_SIZE(spic1_1_pins),
},
{
    .name = "spic2_0_wp_hold",
    .pins = spic2_0_wp_hold_pins,
    .npins = ARRAY_SIZE(spic2_0_wp_hold_pins),
},
{
    .name = "spic2_0",
    .pins = spic2_0_pins,
    .npins = ARRAY_SIZE(spic2_0_pins),
},
{
    .name = "tdm_0_out_mclk_bclk_ws",
    .pins = tdm_0_out_mclk_bclk_ws_pins,
    .npins = ARRAY_SIZE(tdm_0_out_mclk_bclk_ws_pins),
},
{
    .name = "tdm_0_in_mclk_bclk_ws",
    .pins = tdm_0_in_mclk_bclk_ws_pins,
    .npins = ARRAY_SIZE(tdm_0_in_mclk_bclk_ws_pins),
},
{
    .name = "tdm_0_out_data",
    .pins = tdm_0_out_data_pins,
    .npins = ARRAY_SIZE(tdm_0_out_data_pins),
},
{
    .name = "tdm_0_in_data",
    .pins = tdm_0_in_data_pins,
    .npins = ARRAY_SIZE(tdm_0_in_data_pins),
},
{
    .name = "tdm_1_out_mclk_bclk_ws",
    .pins = tdm_1_out_mclk_bclk_ws_pins,
    .npins = ARRAY_SIZE(tdm_1_out_mclk_bclk_ws_pins),
},
{
    .name = "tdm_1_in_mclk_bclk_ws",
    .pins = tdm_1_in_mclk_bclk_ws_pins,
    .npins = ARRAY_SIZE(tdm_1_in_mclk_bclk_ws_pins),
},
{
    .name = "tdm_1_out_data",
    .pins = tdm_1_out_data_pins,
    .npins = ARRAY_SIZE(tdm_1_out_data_pins),
},
{
    .name = "tdm_1_in_data",
    .pins = tdm_1_in_data_pins,
    .npins = ARRAY_SIZE(tdm_1_in_data_pins),
},
{
    .name = "uart0_0_tx_rx",
    .pins = uart0_0_tx_rx_pins,
    .npins = ARRAY_SIZE(uart0_0_tx_rx_pins),
},
{
    .name = "uart1_0_tx_rx",
    .pins = uart1_0_tx_rx_pins,
    .npins = ARRAY_SIZE(uart1_0_tx_rx_pins),
},
{
    .name = "uart1_0_rts_cts",
    .pins = uart1_0_rts_cts_pins,
    .npins = ARRAY_SIZE(uart1_0_rts_cts_pins),
},
{
    .name = "uart1_1_tx_rx",
    .pins = uart1_1_tx_rx_pins,
    .npins = ARRAY_SIZE(uart1_1_tx_rx_pins),
},
{
    .name = "uart1_1_rts_cts",
    .pins = uart1_1_rts_cts_pins,
    .npins = ARRAY_SIZE(uart1_1_rts_cts_pins),
},
{
    .name = "uart2_0_tx_rx",
    .pins = uart2_0_tx_rx_pins,
    .npins = ARRAY_SIZE(uart2_0_tx_rx_pins),
},
{
    .name = "uart2_0_rts_cts",
    .pins = uart2_0_rts_cts_pins,
    .npins = ARRAY_SIZE(uart2_0_rts_cts_pins),
},
{
    .name = "uart2_1_tx_rx",
    .pins = uart2_1_tx_rx_pins,
    .npins = ARRAY_SIZE(uart2_1_tx_rx_pins),
},
{
    .name = "uart2_1_rts_cts",
    .pins = uart2_1_rts_cts_pins,
    .npins = ARRAY_SIZE(uart2_1_rts_cts_pins),
},
{
    .name = "uart2_2_tx_rx",
    .pins = uart2_2_tx_rx_pins,
    .npins = ARRAY_SIZE(uart2_2_tx_rx_pins),
},
{
    .name = "uart2_2_rts_cts",
    .pins = uart2_2_rts_cts_pins,
    .npins = ARRAY_SIZE(uart2_2_rts_cts_pins),
},
{
    .name = "uart2_3_tx_rx",
    .pins = uart2_3_tx_rx_pins,
    .npins = ARRAY_SIZE(uart2_3_tx_rx_pins),
},
{
    .name = "uart3_0_tx_rx",
    .pins = uart3_0_tx_rx_pins,
    .npins = ARRAY_SIZE(uart3_0_tx_rx_pins),
},
{
    .name = "uart3_1_tx_rx",
    .pins = uart3_1_tx_rx_pins,
    .npins = ARRAY_SIZE(uart3_1_tx_rx_pins),
},
{
    .name = "uart3_1_rts_cts",
    .pins = uart3_1_rts_cts_pins,
    .npins = ARRAY_SIZE(uart3_1_rts_cts_pins),
},
{
    .name = "uart4_0_tx_rx",
    .pins = uart4_0_tx_rx_pins,
    .npins = ARRAY_SIZE(uart4_0_tx_rx_pins),
},
{
    .name = "uart4_1_tx_rx",
    .pins = uart4_1_tx_rx_pins,
    .npins = ARRAY_SIZE(uart4_1_tx_rx_pins),
},
{
    .name = "uart4_1_rts_cts",
    .pins = uart4_1_rts_cts_pins,
    .npins = ARRAY_SIZE(uart4_1_rts_cts_pins),
},
{
    .name = "uart4_2_tx_rx",
    .pins = uart4_2_tx_rx_pins,
    .npins = ARRAY_SIZE(uart4_2_tx_rx_pins),
},
{
    .name = "uart4_2_rts_cts",
    .pins = uart4_2_rts_cts_pins,
    .npins = ARRAY_SIZE(uart4_2_rts_cts_pins),
},
{
    .name = "watchdog",
    .pins = watchdog_pins,
    .npins = ARRAY_SIZE(watchdog_pins),
}
};
#define MT7622_NUM_GROUPS ARRAY_SIZE(mt7622_groups)

static const char *const emmc_groups[] = {"emmc"};
static const char *const emmc_rst_groups[] = {"emmc_rst"};
static const char *const esw_groups[] = {"esw"};
static const char *const esw_p0_p1_groups[] = {"esw_p0_p1"};
static const char *const esw_p2_p3_p4_groups[] = {"esw_p2_p3_p4"};
static const char *const rgmii_via_esw_groups[] = {"rgmii_via_esw"};
static const char *const rgmii_via_gmac1_groups[] = {"rgmii_via_gmac1"};
static const char *const rgmii_via_gmac2_groups[] = {"rgmii_via_gmac2"};
static const char *const mdc_mdio4_groups[] = {"mdc_mdio"};

static const char *const i2c0_groups[] = {"i2c0"};
static const char *const i2c1_0_groups[] = {"i2c1_0"};
static const char *const i2c1_1_groups[] = {"i2c1_1"};
static const char *const i2c1_2_groups[] = {"i2c1_2"};
static const char *const i2c2_0_groups[] = {"i2c2_0"};
static const char *const i2c2_1_groups[] = {"i2c2_1"};
static const char *const i2c2_2_groups[] = {"i2c2_2"};
static const char *const i2s_in_mclk_bclk_ws_groups[] = {"i2s_in_mclk_bclk_ws"};

static const char *const i2s1_in_data_groups[] = {"i2s1_in_data"};
static const char *const i2s2_in_data_groups[] = {"i2s2_in_data"};
static const char *const i2s3_in_data_groups[] = {"i2s3_in_data"};
static const char *const i2s4_in_data_groups[] = {"i2s4_in_data"};
static const char *const i2s_out_mclk_bclk_ws_groups[] = {"i2s_out_mclk_bclk_ws"};
static const char *const i2s1_out_data_groups[] = {"i2s1_out_data"};
static const char *const i2s2_out_data_groups[] = {"i2s2_out_data"};
static const char *const i2s3_out_data_groups[] = {"i2s3_out_data"};
static const char *const i2s4_out_data_groups[] = {"i2s4_out_data"};

static const char *const ir_0_tx_groups[] = {"ir_0_tx"};
static const char *const ir_1_tx_groups[] = {"ir_1_tx"};
static const char *const ir_2_tx_groups[] = {"ir_2_tx"};
static const char *const ir_0_rx_groups[] = {"ir_0_rx"};
static const char *const ir_1_rx_groups[] = {"ir_1_rx"};
static const char *const ir_2_rx_groups[] = {"ir_2_rx"};

static const char *const ephy_leds_groups[] = {"ephy_leds"};
static const char *const ephy0_led_groups[] = {"ephy0_led"};
static const char *const ephy1_led_groups[] = {"ephy1_led"};
static const char *const ephy2_led_groups[] = {"ephy2_led"};
static const char *const ephy3_led_groups[] = {"ephy3_led"};
static const char *const ephy4_led_groups[] = {"ephy4_led"};
static const char *const wled_groups[] = {"wled"};

static const char *const par_nand_groups[] = {"par_nand"};
static const char *const snfi_groups[] = {"snfi"};
static const char *const spi_nor_groups[] = {"spi_nor"};

static const char *const pcie0_0_waken_groups[] = {"pcie0_0_waken"};
static const char *const pcie0_1_waken_groups[] = {"pcie0_1_waken"};
static const char *const pcie1_0_waken_groups[] = {"pcie1_0_waken"};
static const char *const pcie0_0_clkreq_groups[] = {"pcie0_0_clkreq"};
static const char *const pcie0_1_clkreq_groups[] = {"pcie0_1_clkreq"};
static const char *const pcie1_0_clkreq_groups[] = {"pcie1_0_clkreq"};
static const char *const pcie0_pad_perst_groups[] = {"pcie0_pad_perst"};
static const char *const pcie1_pad_perst_groups[] = {"pcie1_pad_perst"};

static const char *const pmic_bus_groups[] = {"pmic_bus"};

static const char *const pwm_ch1_0_groups[] = {"pwm_ch1_0"};
static const char *const pwm_ch1_1_groups[] = {"pwm_ch1_1"};
static const char *const pwm_ch1_2_groups[] = {"pwm_ch1_2"};
static const char *const pwm_ch2_0_groups[] = {"pwm_ch2_0"};
static const char *const pwm_ch2_1_groups[] = {"pwm_ch2_1"};
static const char *const pwm_ch2_2_groups[] = {"pwm_ch2_2"};
static const char *const pwm_ch3_0_groups[] = {"pwm_ch3_0"};
static const char *const pwm_ch3_1_groups[] = {"pwm_ch3_1"};
static const char *const pwm_ch3_2_groups[] = {"pwm_ch3_2"};
static const char *const pwm_ch4_0_groups[] = {"pwm_ch4_0"};
static const char *const pwm_ch4_1_groups[] = {"pwm_ch4_1"};
static const char *const pwm_ch4_2_groups[] = {"pwm_ch4_2"};
static const char *const pwm_ch4_3_groups[] = {"pwm_ch4_3"};
static const char *const pwm_ch5_0_groups[] = {"pwm_ch5_0"};
static const char *const pwm_ch5_1_groups[] = {"pwm_ch5_1"};
static const char *const pwm_ch5_2_groups[] = {"pwm_ch5_2"};
static const char *const pwm_ch6_0_groups[] = {"pwm_ch6_0"};
static const char *const pwm_ch6_1_groups[] = {"pwm_ch6_1"};
static const char *const pwm_ch6_2_groups[] = {"pwm_ch6_2"};
static const char *const pwm_ch6_3_groups[] = {"pwm_ch6_3"};
static const char *const pwm_ch7_0_groups[] = {"pwm_ch7_0"};
static const char *const pwm_ch7_1_groups[] = {"pwm_ch7_1"};
static const char *const pwm_ch7_2_groups[] = {"pwm_ch7_2"};

static const char *const sd_0_groups[] = {"sd_0"};
static const char *const sd_1_groups[] = {"sd_1"};
static const char *const spic0_0_groups[] = {"spic0_0"};
static const char *const spic0_1_groups[] = {"spic0_1"};
static const char *const spic1_0_groups[] = {"spic1_0"};
static const char *const spic1_1_groups[] = {"spic1_1"};
static const char *const spic2_0_wp_hold_groups[] = {"spic2_0_wp_hold"};
static const char *const spic2_0_groups[] = {"spic2_0"};

static const char *const tdm_0_out_mclk_bclk_ws_groups[] = {"tdm_0_out_mclk_bclk_ws"};
static const char *const tdm_0_in_mclk_bclk_ws_groups[] = {"tdm_0_in_mclk_bclk_ws"};
static const char *const tdm_0_out_data_groups[] = {"tdm_0_out_data"};
static const char *const tdm_0_in_data_groups[] = {"tdm_0_in_data"};
static const char *const tdm_1_out_mclk_bclk_ws_groups[] = {"tdm_1_out_mclk_bclk_ws"};
static const char *const tdm_1_in_mclk_bclk_ws_groups[] = {"tdm_1_in_mclk_bclk_ws"};
static const char *const tdm_1_out_data_groups[] = {"tdm_1_out_data"};
static const char *const tdm_1_in_data_groups[] = {"tdm_1_in_data"};

static const char *const uart0_0_tx_rx_groups[] = {"uart0_0_tx_rx"};
static const char *const uart1_0_tx_rx_groups[] = {"uart1_0_tx_rx"};
static const char *const uart1_0_rts_cts_groups[] = {"uart1_0_rts_cts"};
static const char *const uart1_1_tx_rx_groups[] = {"uart1_1_tx_rx"};
static const char *const uart1_1_rts_cts_groups[] = {"uart1_1_rts_cts"};
static const char *const uart2_0_tx_rx_groups[] = {"uart2_0_tx_rx"};
static const char *const uart2_0_rts_cts_groups[] = {"uart2_0_rts_cts"};
static const char *const uart2_1_tx_rx_groups[] = {"uart2_1_tx_rx"};

static const char *const uart2_1_rts_cts_groups[] = {"uart2_1_rts_cts"};
static const char *const uart2_2_tx_rx_groups[] = {"uart2_2_tx_rx"};
static const char *const uart2_2_rts_cts_groups[] = {"uart2_2_rts_cts"};
static const char *const uart2_3_tx_rx_groups[] = {"uart2_3_tx_rx"};
static const char *const uart3_0_tx_rx_groups[] = {"uart3_0_tx_rx"};
static const char *const uart3_1_tx_rx_groups[] = {"uart3_1_tx_rx"};
static const char *const uart3_1_rts_cts_groups[] = {"uart3_1_rts_cts"};
static const char *const uart4_0_tx_rx_groups[] = {"uart4_0_tx_rx"};

static const char *const uart4_1_tx_rx_groups[] = {"uart4_1_tx_rx"};
static const char *const uart4_1_rts_cts_groups[] = {"uart4_1_rts_cts"};
static const char *const uart4_2_tx_rx_groups[] = {"uart4_2_tx_rx"};
static const char *const uart4_2_rts_cts_groups[] = {"uart4_2_rts_cts"};

static const char *const watchdog_groups[] = {"watchdog"};


static const struct pinctrl_function mt7622_functions[] = {
{
    .name = "emmc",
    .groups = emmc_groups,
    .ngroups = ARRAY_SIZE(emmc_groups),
},
{
    .name = "emmc",
    .groups = emmc_rst_groups,
    .ngroups = ARRAY_SIZE(emmc_rst_groups),
},
{
    .name = "eth",
    .groups = esw_groups,
    .ngroups = ARRAY_SIZE(esw_groups),
},
{
    .name = "eth",
    .groups = esw_p0_p1_groups,
    .ngroups = ARRAY_SIZE(esw_p0_p1_groups),
},
{
    .name = "eth",
    .groups = esw_p2_p3_p4_groups,
    .ngroups = ARRAY_SIZE(esw_p2_p3_p4_groups),
},
{
    .name = "eth",
    .groups = rgmii_via_esw_groups,
    .ngroups = ARRAY_SIZE(rgmii_via_esw_groups),
},
{
    .name = "eth",
    .groups = rgmii_via_gmac1_groups,
    .ngroups = ARRAY_SIZE(rgmii_via_gmac1_groups),
},
{
    .name = "eth",
    .groups = rgmii_via_gmac2_groups,
    .ngroups = ARRAY_SIZE(rgmii_via_gmac2_groups),
},
{
    .name = "eth",
    .groups = mdc_mdio4_groups,
    .ngroups = ARRAY_SIZE(mdc_mdio4_groups),
},
{
    .name = "i2c",
    .groups = i2c0_groups,
    .ngroups = ARRAY_SIZE(i2c0_groups),
},
{
    .name = "i2c",
    .groups = i2c1_0_groups,
    .ngroups = ARRAY_SIZE(i2c1_0_groups),
},
{
    .name = "i2c",
    .groups = i2c1_1_groups,
    .ngroups = ARRAY_SIZE(i2c1_1_groups),
},
{
    .name = "i2c",
    .groups = i2c1_2_groups,
    .ngroups = ARRAY_SIZE(i2c1_2_groups),
},
{
    .name = "i2c",
    .groups = i2c2_0_groups,
    .ngroups = ARRAY_SIZE(i2c2_0_groups),
},
{
    .name = "i2c",
    .groups = i2c2_1_groups,
    .ngroups = ARRAY_SIZE(i2c2_1_groups),
},
{
    .name = "i2c",
    .groups = i2c2_2_groups,
    .ngroups = ARRAY_SIZE(i2c2_2_groups),
},
{
    .name = "i2c",
    .groups = i2s_in_mclk_bclk_ws_groups,
    .ngroups = ARRAY_SIZE(i2s_in_mclk_bclk_ws_groups),
},
{
    .name = "i2c",
    .groups = i2s1_in_data_groups,
    .ngroups = ARRAY_SIZE(i2s1_in_data_groups),
},
{
    .name = "i2c",
    .groups = i2s2_in_data_groups,
    .ngroups = ARRAY_SIZE(i2s2_in_data_groups),
},
{
    .name = "i2c",
    .groups = i2s3_in_data_groups,
    .ngroups = ARRAY_SIZE(i2s3_in_data_groups),
},
{
    .name = "i2c",
    .groups = i2s4_in_data_groups,
    .ngroups = ARRAY_SIZE(i2s4_in_data_groups),
},
{
    .name = "i2c",
    .groups = i2s_out_mclk_bclk_ws_groups,
    .ngroups = ARRAY_SIZE(i2s_out_mclk_bclk_ws_groups),
},
{
    .name = "i2c",
    .groups = i2s1_out_data_groups,
    .ngroups = ARRAY_SIZE(i2s1_out_data_groups),
},
{
    .name = "i2c",
    .groups = i2s2_out_data_groups,
    .ngroups = ARRAY_SIZE(i2s2_out_data_groups),
},
{
    .name = "i2c",
    .groups = i2s3_out_data_groups,
    .ngroups = ARRAY_SIZE(i2s3_out_data_groups),
},
{
    .name = "i2c",
    .groups = i2s4_out_data_groups,
    .ngroups = ARRAY_SIZE(i2s4_out_data_groups),
},
{
    .name = "ir",
    .groups = ir_0_tx_groups,
    .ngroups = ARRAY_SIZE(ir_0_tx_groups),
},
{
    .name = "ir",
    .groups = ir_1_tx_groups,
    .ngroups = ARRAY_SIZE(ir_1_tx_groups),
},
{
    .name = "ir",
    .groups = ir_2_tx_groups,
    .ngroups = ARRAY_SIZE(ir_2_tx_groups),
},
{
    .name = "ir",
    .groups = ir_0_rx_groups,
    .ngroups = ARRAY_SIZE(ir_0_rx_groups),
},
{
    .name = "ir",
    .groups = ir_1_rx_groups,
    .ngroups = ARRAY_SIZE(ir_1_rx_groups),
},
{
    .name = "ir",
    .groups = ir_2_rx_groups,
    .ngroups = ARRAY_SIZE(ir_2_rx_groups),
},
{
    .name = "led",
    .groups = ephy_leds_groups,
    .ngroups = ARRAY_SIZE(ephy_leds_groups),
},
{
    .name = "led",
    .groups = ephy0_led_groups,
    .ngroups = ARRAY_SIZE(ephy0_led_groups),
},
{
    .name = "led",
    .groups = ephy1_led_groups,
    .ngroups = ARRAY_SIZE(ephy1_led_groups),
},
{
    .name = "led",
    .groups = ephy2_led_groups,
    .ngroups = ARRAY_SIZE(ephy2_led_groups),
},
{
    .name = "led",
    .groups = ephy3_led_groups,
    .ngroups = ARRAY_SIZE(ephy3_led_groups),
},
{
    .name = "led",
    .groups = ephy4_led_groups,
    .ngroups = ARRAY_SIZE(ephy4_led_groups),
},
{
    .name = "led",
    .groups = wled_groups,
    .ngroups = ARRAY_SIZE(wled_groups),
},
{
    .name = "flash",
    .groups = par_nand_groups,
    .ngroups = ARRAY_SIZE(par_nand_groups),
},
{
    .name = "flash",
    .groups = snfi_groups,
    .ngroups = ARRAY_SIZE(snfi_groups),
},
{
    .name = "flash",
    .groups = spi_nor_groups,
    .ngroups = ARRAY_SIZE(spi_nor_groups),
},
{
    .name = "pcie",
    .groups = pcie0_0_waken_groups,
    .ngroups = ARRAY_SIZE(pcie0_0_waken_groups),
},
{
    .name = "pcie",
    .groups = pcie0_1_waken_groups,
    .ngroups = ARRAY_SIZE(pcie0_1_waken_groups),
},
{
    .name = "pcie",
    .groups = pcie1_0_waken_groups,
    .ngroups = ARRAY_SIZE(pcie1_0_waken_groups),
},
{
    .name = "pcie",
    .groups = pcie0_0_clkreq_groups,
    .ngroups = ARRAY_SIZE(pcie0_0_clkreq_groups),
},
{
    .name = "pcie",
    .groups = pcie0_1_clkreq_groups,
    .ngroups = ARRAY_SIZE(pcie0_1_clkreq_groups),
},
{
    .name = "pcie",
    .groups = pcie1_0_clkreq_groups,
    .ngroups = ARRAY_SIZE(pcie1_0_clkreq_groups),
},
{
    .name = "pcie",
    .groups = pcie0_pad_perst_groups,
    .ngroups = ARRAY_SIZE(pcie0_pad_perst_groups),
},
{
    .name = "pcie",
    .groups = pcie1_pad_perst_groups,
    .ngroups = ARRAY_SIZE(pcie1_pad_perst_groups),
},
{
    .name = "pmic",
    .groups = pmic_bus_groups,
    .ngroups = ARRAY_SIZE(pmic_bus_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch1_0_groups,
    .ngroups = ARRAY_SIZE(pwm_ch1_0_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch1_1_groups,
    .ngroups = ARRAY_SIZE(pwm_ch1_1_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch1_2_groups,
    .ngroups = ARRAY_SIZE(pwm_ch1_2_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch2_0_groups,
    .ngroups = ARRAY_SIZE(pwm_ch2_0_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch2_1_groups,
    .ngroups = ARRAY_SIZE(pwm_ch2_1_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch2_2_groups,
    .ngroups = ARRAY_SIZE(pwm_ch2_2_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch3_0_groups,
    .ngroups = ARRAY_SIZE(pwm_ch3_0_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch3_1_groups,
    .ngroups = ARRAY_SIZE(pwm_ch3_1_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch3_2_groups,
    .ngroups = ARRAY_SIZE(pwm_ch3_2_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch4_0_groups,
    .ngroups = ARRAY_SIZE(pwm_ch4_0_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch4_1_groups,
    .ngroups = ARRAY_SIZE(pwm_ch4_1_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch4_2_groups,
    .ngroups = ARRAY_SIZE(pwm_ch4_2_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch4_3_groups,
    .ngroups = ARRAY_SIZE(pwm_ch4_3_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch5_0_groups,
    .ngroups = ARRAY_SIZE(pwm_ch5_0_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch5_1_groups,
    .ngroups = ARRAY_SIZE(pwm_ch5_1_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch5_2_groups,
    .ngroups = ARRAY_SIZE(pwm_ch5_2_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch6_0_groups,
    .ngroups = ARRAY_SIZE(pwm_ch6_0_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch6_1_groups,
    .ngroups = ARRAY_SIZE(pwm_ch6_1_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch6_2_groups,
    .ngroups = ARRAY_SIZE(pwm_ch6_2_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch6_3_groups,
    .ngroups = ARRAY_SIZE(pwm_ch6_3_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch7_0_groups,
    .ngroups = ARRAY_SIZE(pwm_ch7_0_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch7_1_groups,
    .ngroups = ARRAY_SIZE(pwm_ch7_1_groups),
},
{
    .name = "pwm",
    .groups = pwm_ch7_2_groups,
    .ngroups = ARRAY_SIZE(pwm_ch7_2_groups),
},
{
    .name = "sd",
    .groups = sd_0_groups,
    .ngroups = ARRAY_SIZE(sd_0_groups),
},
{
    .name = "sd",
    .groups = sd_1_groups,
    .ngroups = ARRAY_SIZE(sd_1_groups),
},
{
    .name = "spi",
    .groups = spic0_0_groups,
    .ngroups = ARRAY_SIZE(spic0_0_groups),
},
{
    .name = "spi",
    .groups = spic0_1_groups,
    .ngroups = ARRAY_SIZE(spic0_1_groups),
},
{
    .name = "spi",
    .groups = spic1_0_groups,
    .ngroups = ARRAY_SIZE(spic1_0_groups),
},
{
    .name = "spi",
    .groups = spic1_1_groups,
    .ngroups = ARRAY_SIZE(spic1_1_groups),
},
{
    .name = "spi",
    .groups = spic2_0_wp_hold_groups,
    .ngroups = ARRAY_SIZE(spic2_0_wp_hold_groups),
},
{
    .name = "spi",
    .groups = spic2_0_groups,
    .ngroups = ARRAY_SIZE(spic2_0_groups),
},
{
    .name = "tdm",
    .groups = tdm_0_out_mclk_bclk_ws_groups,
    .ngroups = ARRAY_SIZE(tdm_0_out_mclk_bclk_ws_groups),
},
{
    .name = "tdm",
    .groups = tdm_0_in_mclk_bclk_ws_groups,
    .ngroups = ARRAY_SIZE(tdm_0_in_mclk_bclk_ws_groups),
},
{
    .name = "tdm",
    .groups = tdm_0_out_data_groups,
    .ngroups = ARRAY_SIZE(tdm_0_out_data_groups),
},
{
    .name = "tdm",
    .groups = tdm_0_in_data_groups,
    .ngroups = ARRAY_SIZE(tdm_0_in_data_groups),
},
{
    .name = "tdm",
    .groups = tdm_1_out_mclk_bclk_ws_groups,
    .ngroups = ARRAY_SIZE(tdm_1_out_mclk_bclk_ws_groups),
},
{
    .name = "tdm",
    .groups = tdm_1_in_mclk_bclk_ws_groups,
    .ngroups = ARRAY_SIZE(tdm_1_in_mclk_bclk_ws_groups),
},
{
    .name = "tdm",
    .groups = tdm_1_out_data_groups,
    .ngroups = ARRAY_SIZE(tdm_1_out_data_groups),
},
{
    .name = "tdm",
    .groups = tdm_1_in_data_groups,
    .ngroups = ARRAY_SIZE(tdm_1_in_data_groups),
},
{
    .name = "uart",
    .groups = uart0_0_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart0_0_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart1_0_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart1_0_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart1_0_rts_cts_groups,
    .ngroups = ARRAY_SIZE(uart1_0_rts_cts_groups),
},
{
    .name = "uart",
    .groups = uart1_1_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart1_1_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart1_1_rts_cts_groups,
    .ngroups = ARRAY_SIZE(uart1_1_rts_cts_groups),
},
{
    .name = "uart",
    .groups = uart2_0_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart2_0_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart2_0_rts_cts_groups,
    .ngroups = ARRAY_SIZE(uart2_0_rts_cts_groups),
},
{
    .name = "uart",
    .groups = uart2_1_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart2_1_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart2_1_rts_cts_groups,
    .ngroups = ARRAY_SIZE(uart2_1_rts_cts_groups),
},
{
    .name = "uart",
    .groups = uart2_2_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart2_2_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart2_2_rts_cts_groups,
    .ngroups = ARRAY_SIZE(uart2_2_rts_cts_groups),
},
{
    .name = "uart",
    .groups = uart2_3_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart2_3_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart3_0_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart3_0_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart3_1_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart3_1_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart3_1_rts_cts_groups,
    .ngroups = ARRAY_SIZE(uart3_1_rts_cts_groups),
},
{
    .name = "uart",
    .groups = uart4_0_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart4_0_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart4_1_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart4_1_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart4_1_rts_cts_groups,
    .ngroups = ARRAY_SIZE(uart4_1_rts_cts_groups),
},
{
    .name = "uart",
    .groups = uart4_2_tx_rx_groups,
    .ngroups = ARRAY_SIZE(uart4_2_tx_rx_groups),
},
{
    .name = "uart",
    .groups = uart4_2_rts_cts_groups,
    .ngroups = ARRAY_SIZE(uart4_2_rts_cts_groups),
},
{
    .name = "watchdog",
    .groups = watchdog_groups,
    .ngroups = ARRAY_SIZE(watchdog_groups),
}
};
#define MT7622_NUM_FUNCTIONS ARRAY_SIZE(mt7622_functions)

static inline uint32_t mt7622_pinctrl_read_4(struct mt7622_pinctrl_softc *sc,
                                             bus_size_t offset) {
    return bus_read_4(sc->mem_res, offset);
}

static inline void mt7622_pinctrl_write_4(struct mt7622_pinctrl_softc *sc,
                                          bus_size_t offset, uint32_t value) {
    bus_write_4(sc->mem_res, offset, value);
}*/

struct mt7622_pinmux_desc {
    const char *funcs[8];
    bus_size_t reg_offset;
    int shift;
};

struct mt7622_pinctrl_softc {
    device_t sc_dev;
    device_t sc_busdev;
    struct resource *mem_res;
    int mem_rid;
    const struct mt7622_pinmux_desc *pixmux;
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

static int
mt7622_pinctrl_configure(device_t dev, phandle_t cfgxref) {
    //struct mt7622_pinctrl_softc *sc = device_get_softc(dev);
    phandle_t node = OF_node_from_xref(cfgxref);
    phandle_t child;
    char name[64], function[64], groups[64];

    if (node <= 0)
        return (ENXIO);

    for (child = OF_child(node); child != 0; child = OF_peer(child)) {
        if (OF_getprop(child, "name", name, sizeof(name)) <= 0)
            continue;

        if (strncmp(name, "mux", 3) == 0) {
            if (OF_getprop(child, "function", function, sizeof(function)) > 0 &&
                    OF_getprop(child, "groups", groups, sizeof(groups)) > 0) {

                device_printf(dev, "Name: %s Function: %s, Groups: %s\n",name, function, groups);

                /*int func_index = -1;
                int group_index = -1;

                for (int i = 0; i < sc->nfunctions; i++) {
                    if (strcmp(sc->functions[i].name, function) == 0) {
                        func_index = i;
                        break;
                    }
                }

                for (int i = 0; i < sc->ngroups; i++) {
                    if (strcmp(sc->groups[i].name, groups) == 0) {
                        group_index = i;
                        break;
                    }
                }

                device_printf(dev, "Func_index: %d , Group_index: %d \n", func_index, group_index);
                if (func_index < 0 || group_index < 0) {
                    device_printf(dev, "Unknown function or group\n");
                    continue;
                }

                const struct pinctrl_pin_group *grp = &sc->groups[group_index];
                uint32_t mux_val = func_index;

                for (unsigned i = 0; i < grp->npins; i++) {
                    uint32_t pin = grp->pins[i];
                    uint32_t offset = 0x300 + (pin / 8) * 0x10;
                    uint32_t shift = (pin % 8) * 4;
                    uint32_t val = mt7622_pinctrl_read_4(sc, offset);

                    val &= ~(0xf << shift);
                    val |= (mux_val << shift);
                    mt7622_pinctrl_write_4(sc, offset, val);

                    device_printf(dev, "Pin %u mux set to %u (reg 0x%x)\n", pin, mux_val,
                                  offset);
                }*/
            }
        }
    }

    return 0;
}

static int
mtk_pinctrl_probe(device_t dev) {
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
        return (ENXIO);

    device_set_desc(dev, "Mediatek 7622 pinctrl configuration");
    return (BUS_PROBE_DEFAULT);
}

static int
mtk_pinctrl_attach(device_t dev) {
    struct mt7622_pinctrl_softc *sc = device_get_softc(dev);
    sc->sc_dev = dev;

    /* Map memory resource */
    sc->mem_rid = 0;
    sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid, RF_ACTIVE);
    if (sc->mem_res == NULL) {
        device_printf(dev, "Could not map memory resource\n");
        return (ENXIO);
    }

    /*sc->pins = mt7622_pins;
    sc->npins = MT7622_NUM_PINS;
    sc->groups = mt7622_groups;
    sc->ngroups = MT7622_NUM_GROUPS;
    sc->functions = mt7622_functions;
    sc->nfunctions = MT7622_NUM_FUNCTIONS;*/
    sc->pixmux = pinmux;

    fdt_pinctrl_register(dev, NULL);
    fdt_pinctrl_configure_tree(dev);

    return (0);
}

static int mtk_pinctrl_detach(device_t dev) {
    struct mt7622_pinctrl_softc *sc = device_get_softc(dev);
    if (sc->mem_res) {
        bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
        sc->mem_res = NULL;
    }

    return (0);
}

static device_method_t mt7622_pinmux_methods[] = {
    DEVMETHOD(device_probe, mtk_pinctrl_probe),
    DEVMETHOD(device_attach, mtk_pinctrl_attach),
    DEVMETHOD(device_detach, mtk_pinctrl_detach),
    DEVMETHOD(fdt_pinctrl_configure, mt7622_pinctrl_configure), DEVMETHOD_END};

static DEFINE_CLASS_0(pinmux, mt7622_pinmux_driver, mt7622_pinmux_methods,
                      sizeof(struct mt7622_pinctrl_softc));
EARLY_DRIVER_MODULE(mt7622_pinmux, simplebus, mt7622_pinmux_driver, NULL, NULL,
                    71);
