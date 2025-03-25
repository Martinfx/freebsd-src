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
};

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
}};
#define MT7622_NUM_GROUPS ARRAY_SIZE(mt7622_groups)

static const char *const emmc_groups[] = {"emmc"};
static const char *const emmc_rst_groups[] = {"emmc_rst"};

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
}};
#define MT7622_NUM_FUNCTIONS ARRAY_SIZE(mt7622_functions)

static inline uint32_t mt7622_pinctrl_read_4(struct mt7622_pinctrl_softc *sc,
                                             bus_size_t offset) {
    return bus_read_4(sc->mem_res, offset);
}

static inline void mt7622_pinctrl_write_4(struct mt7622_pinctrl_softc *sc,
                                          bus_size_t offset, uint32_t value) {
    bus_write_4(sc->mem_res, offset, value);
}

static int
mt7622_pinctrl_configure(device_t dev, phandle_t cfgxref) {
    struct mt7622_pinctrl_softc *sc = device_get_softc(dev);
    phandle_t node = OF_node_from_xref(cfgxref);
    phandle_t child;
    char name[32], function[32], groups[32];

    if (node <= 0)
        return (ENXIO);

    for (child = OF_child(node); child != 0; child = OF_peer(child)) {
        if (OF_getprop(child, "name", name, sizeof(name)) <= 0)
            continue;

        if (strncmp(name, "mux", 3) == 0) {
            if (OF_getprop(child, "function", function, sizeof(function)) > 0 &&
                    OF_getprop(child, "groups", groups, sizeof(groups)) > 0) {

                device_printf(dev, "Function: %s, Groups: %s\n", function, groups);

                int func_index = -1;
                int group_index = -1;

                for (int i = 0; i < sc->nfunctions; i++) {
                    device_printf(dev, "sc->functions[i].name: %s, function: %s\n", sc->functions[i].name, function);
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
                }
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
    pcell_t gpio_ranges[4];
    int error;
    phandle_t node = ofw_bus_get_node(dev);
    struct mt7622_pinctrl_softc *sc = device_get_softc(dev);
    sc->sc_dev = dev;

    /* Map memory resource */
    sc->mem_rid = 0;
    sc->mem_res =
            bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid, RF_ACTIVE);
    if (sc->mem_res == NULL) {
        device_printf(dev, "Could not map memory resource\n");
        return (ENXIO);
    }

    sc->pins = mt7622_pins;
    sc->npins = MT7622_NUM_PINS;
    sc->groups = mt7622_groups;
    sc->ngroups = MT7622_NUM_GROUPS;
    sc->functions = mt7622_functions;
    sc->nfunctions = MT7622_NUM_FUNCTIONS;

    error = OF_getencprop(node, "gpio-ranges", gpio_ranges, sizeof(gpio_ranges));

    if (error == -1) {
        device_printf(dev, "failed to get gpio-ranges\n");
        goto error;
    }

    sc->sc_ngpios = gpio_ranges[3];

    if (sc->sc_ngpios == 0) {
        device_printf(dev, "no GPIOs\n");
        goto error;
    }

    sc->sc_busdev = gpiobus_attach_bus(dev);
    if (sc->sc_busdev == NULL) {
        device_printf(dev, "failed to attach gpiobus\n");

        goto error;
    }

    fdt_pinctrl_register(dev, NULL);
    fdt_pinctrl_configure_tree(dev);

    if (!OF_hasprop(node, "interrupt-controller"))
        return (0);

    sc->sc_irqs = mallocarray(sc->sc_ngpios, sizeof(*sc->sc_irqs), M_DEVBUF,
                              M_ZERO | M_WAITOK);
    intr_pic_register(dev, OF_xref_from_node(ofw_bus_get_node(dev)));

    return (0);

error:
    bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
    sc->mem_res = NULL;
    return (ENXIO);
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
