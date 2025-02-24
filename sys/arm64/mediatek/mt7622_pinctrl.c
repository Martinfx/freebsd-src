#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#define MT7622_PINMUX_FUNC_MAX 32

struct mt7622_pinctrl_function {
    const char *name;
    uint32_t mode;
};

struct mt7622_pinctrl_group {
    const char *name;
    uint32_t pins[MT7622_PINMUX_FUNC_MAX];
    uint32_t functions[MT7622_PINMUX_FUNC_MAX];
};

static struct mt7622_pinctrl_function mt7622_functions[] = {
    { "gpio", 0 },
    { "emmc", 1 },
    { "eth", 2 },
    { "i2c", 3 },
    { "i2s", 4 },
    { "ir", 5 },
    { "led", 6 },
    { "flash", 7 },
    { "pcie", 8 },
    { "pmic", 9 },
    { "pwm", 10 },
    { "sd", 11 },
    { "tdm", 13 },
    { "uart", 14 },
    { "watchdog", 15 },
    { "wifi", 16 },
    { "nand", 9 },
    { "i2s", 4 },
    { "mdio", 11 },
    { "pmic", 17 },
    { "ir", 18 },
    { "sd", 19 },
    { "pcie0", 16 },
    { "pcie1", 17 },
    { "mmc0", 18 },
    { "mmc1", 19 },
    { "ir-receiver", 20 },
    { "antsel", 20 }
};

static struct mt7622_pinctrl_group mt7622_groups[] = {
    { "mdc_mdio", {23, 24}, {3} },
    { "i2c_0", {19, 20}, {4} },
    { "i2c_1", {53, 54}, {4} },
    { "ephy_leds", {12, 13, 14, 15, 16, 17, 18}, {7} },
    { "snfi", {62, 63, 64, 65, 66, 67}, {8} },
    { "spi_nor", {62, 63, 64, 65, 66, 67}, {8} },
    { "pcie_pereset", {51}, {9} },
    { "pcie_wake", {55}, {9} },
    { "pcie_clkreq", {56}, {9} },
    { "pwm_0", {52}, {11} },
    { "pwm_1", {61}, {11} },
    { "spi_0", {21, 22, 23, 24}, {13} },
    { "spi_1", {62, 63, 64, 65}, {13} },
    { "spi_wp", {66}, {13} },
    { "spi_hold", {67}, {13} },
    { "uart0_txd_rxd", {68, 69}, {15} },
    { "uart1_0_txd_rxd", {25, 26}, {15} },
    { "uart1_0_cts_rts", {27, 28}, {15} },
    { "uart2_0_txd_rxd", {29, 30}, {15} },
    { "uart2_0_cts_rts", {31, 32}, {15} },
    { "watchdog", {11}, {16} },
    { "wifi", {70, 71, 72, 73, 74, 75, 76, 77, 78}, {17} },
    { "gpio_a", {0}, {0} },
    { "gpio_b", {22}, {0} },
    { "gpio_d", {77}, {0} },
    { "gpio_e", {102}, {0} },
    { "pwm_2", {95}, {11} },
    { "pwm_3", {96}, {11} },
    { "pwm_4", {97}, {11} },
    { "pwm_5", {98}, {11} },
    { "pwm_6", {99}, {11} },
    { "antsel", {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29}, {20} }
};


static struct ofw_compat_data compat_data[] = {
    { "mediatek,mt7622-pinctrl", 1 },
    { NULL, 0 }
};

struct mt7622_pinctrl_softc {
    device_t dev;
    struct resource *mem_res;
    struct resource *eint_res;
    struct mt7622_pinctrl_function *functions;
    struct mt7622_pinctrl_group *groups;
    size_t num_functions;
    size_t num_groups;
};

static int
mt7622_pinctrl_init(struct mt7622_pinctrl_softc *sc)
{
    sc->functions = mt7622_functions;
    sc->groups = mt7622_groups;
    sc->num_functions = sizeof(mt7622_functions) / sizeof(mt7622_functions[0]);
    sc->num_groups = sizeof(mt7622_groups) / sizeof(mt7622_groups[0]);

    device_printf(sc->dev, "MT7622 pinctrl initialized with %zu functions and %zu groups\n",
                  sc->num_functions, sc->num_groups);
    return (0);
}

static int
pinmux_configure(device_t dev, uint32_t phandle)
{
    return (EINVAL);
}


static int
pinmux_probe(device_t dev)
{

    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
        return (ENXIO);

    device_set_desc(dev, "Mediatek pin configuration");
    return (BUS_PROBE_DEFAULT);
}

static int
pinmux_detach(device_t dev)
{
    return (EBUSY);
}

static int
pinmux_attach(device_t dev)
{
    struct mt7622_pinctrl_softc *sc;
    int rid;
    //phandle_t node;

    sc = device_get_softc(dev);
    sc->dev = dev;

    rid = 0;
    sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
                                          RF_ACTIVE);
    if (sc->mem_res == NULL) {
        device_printf(dev, "Cannot allocate memory resources\n");
        return (ENXIO);
    }

    rid = 1;
    sc->eint_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
                                          RF_ACTIVE);
    if (sc->eint_res == NULL) {
        device_printf(dev, "Cannot allocate memory resources\n");
        return (ENXIO);
    }

    mt7622_pinctrl_init(sc);

    fdt_pinctrl_register(dev, NULL);
    if (fdt_pinctrl_configure_tree(dev) != 0) {
        device_printf(dev, "Cannot configure pinctrl device!\n");
    }

    return (0);
}


static device_method_t mt7622_pinmux_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,         pinmux_probe),
    DEVMETHOD(device_attach,        pinmux_attach),
    DEVMETHOD(device_detach,        pinmux_detach),

    /* fdt_pinctrl interface */
    DEVMETHOD(fdt_pinctrl_configure,pinmux_configure),

    DEVMETHOD_END
};

static DEFINE_CLASS_0(pinmux, mt7622_pinmux_driver, mt7622_pinmux_methods,
    sizeof(struct mt7622_pinctrl_softc));
EARLY_DRIVER_MODULE(mt7622_pinmux, simplebus, mt7622_pinmux_driver,
    NULL, NULL, 71);
