#include <sys/cdefs.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/malloc.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/fdt/fdt_common.h>
#include <dev/fdt/fdt_pinctrl.h>
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

static struct ofw_compat_data compat_data[] = {
    { "mediatek,mt7622-pinctrl", 1 },
    { NULL, 0 }
};

#define ARRAY_SIZE(x)            (sizeof(x) / sizeof((x)[0]))

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
    const char * const *groups;
    unsigned int ngroups;
};

struct mt7622_pinctrl_fdt_cfg {
    uint32_t pin;
    uint32_t func;
    uint32_t config;
};

struct mt7622_pinctrl_softc {
    device_t        dev;
    struct resource *mem_res;
    int             mem_rid;

    const struct pinctrl_pin_desc *pins;
    unsigned int    npins;


    const struct pinctrl_pin_group *groups;
    unsigned int    ngroups;

    const struct pinctrl_function *functions;
    unsigned int    nfunctions;

    bus_space_tag_t     bst;
    bus_space_handle_t  bsh;
};

static const struct pinctrl_pin_desc mt7622_pins[] = {
    { .number = 0, .name = "GPIO0" },
    { .number = 1, .name = "GPIO1" },
    { .number = 2, .name = "GPIO2" },
    { .number = 3, .name = "GPIO3" },
};
#define MT7622_NUM_PINS ARRAY_SIZE(mt7622_pins)

static const unsigned int uart0_pins[] = { 0, 1 };  /* UART uses GPIO0 and GPIO1 */
static const unsigned int i2c0_pins[] = { 2, 3 };   /* I2C uses GPIO2 and GPIO3 */

static const struct pinctrl_pin_group mt7622_groups[] = {
    {
        .name = "uart0_grp",
        .pins = uart0_pins,
        .npins = ARRAY_SIZE(uart0_pins),
    },
    {
        .name = "i2c0_grp",
        .pins = i2c0_pins,
        .npins = ARRAY_SIZE(i2c0_pins),
    }
};
#define MT7622_NUM_GROUPS ARRAY_SIZE(mt7622_groups)

static const char * const uart_groups[] = { "uart0_grp" };
static const char * const i2c_groups[] = { "i2c0_grp" };

static const struct pinctrl_function mt7622_functions[] = {
    {
        .name = "uart",
        .groups = uart_groups,
        .ngroups = ARRAY_SIZE(uart_groups),
    },
    {
        .name = "i2c",
        .groups = i2c_groups,
        .ngroups = ARRAY_SIZE(i2c_groups),
    }
};
#define MT7622_NUM_FUNCTIONS ARRAY_SIZE(mt7622_functions)

static inline uint32_t
mt7622_pinctrl_read_4(struct mt7622_pinctrl_softc *sc, bus_size_t offset)
{
    return bus_read_4(sc->mem_res, offset);
}

static inline void
mt7622_pinctrl_write_4(struct mt7622_pinctrl_softc *sc, bus_size_t offset, uint32_t value)
{
    bus_write_4(sc->mem_res, offset, value);
}

static int
mt7622_pinctrl_configure(device_t dev, phandle_t cfgxref)
{
    phandle_t node = OF_node_from_xref(cfgxref);
    phandle_t child;
    char name[32];
    char function[32];
    char groups[32];
    uint32_t pins[16];
    int len;

    if (node <= 0) {
        return (ENXIO);
    }

    for (child = OF_child(node); child != 0;	child =	OF_peer(child)) {
        if (OF_getprop(child, "name", name, sizeof(name)) > 0) {
            device_printf(dev, "Name %s\n", name);

            /* Handle mux configuration nodes */
            if (strncmp(name, "mux", 3) == 0) {
                /* Get function and groups properties */
                if (OF_getprop(child, "function", function, sizeof(function)) > 0) {
                    device_printf(dev, "Function: %s\n", function);
                }
                if (OF_getprop(child, "groups", groups, sizeof(groups)) > 0) {
                    device_printf(dev, "Groups: %s\n", groups);
                }
            }
            else if (strncmp(name, "cfg", 3) == 0) {
                len = OF_getprop(child, "pins", pins, sizeof(pins));
                if (len > 0) {
                    device_printf(dev, "Pins: ");
                    for (int i = 0; i < len/4; i++) {
                        device_printf(dev, "%d ", pins[i]);
                    }
                    device_printf(dev, "\n");
                }
            }
        }
    }

    return 0;
}

static int
mtk_pinctrl_probe(device_t dev)
{
    if (!ofw_bus_status_okay(dev))
        return (ENXIO);

    if (!ofw_bus_search_compatible(dev, compat_data)->ocd_data)
        return (ENXIO);

    device_set_desc(dev, "Mediatek 7622 pinctrl configuration");
    return (BUS_PROBE_DEFAULT);
}

static int
mtk_pinctrl_attach(device_t dev)
{
    struct mt7622_pinctrl_softc *sc = device_get_softc(dev);
    sc->dev = dev;

    /* Map memory resource */
    sc->mem_rid = 0;
    sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
        &sc->mem_rid, RF_ACTIVE);
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

    fdt_pinctrl_register(dev, NULL);
    if (fdt_pinctrl_configure_tree(dev) != 0) {
        device_printf(dev, "Cannot configure pinctrl device!\n");
    }
    return (0);
}

static int
mtk_pinctrl_detach(device_t dev)
{
    struct mt7622_pinctrl_softc *sc = device_get_softc(dev);
    if (sc->mem_res) {
        bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
        sc->mem_res = NULL;
    }

    return (0);
}

static device_method_t mt7622_pinmux_methods[] = {
    DEVMETHOD(device_probe,         mtk_pinctrl_probe),
    DEVMETHOD(device_attach,        mtk_pinctrl_attach),
    DEVMETHOD(device_detach,        mtk_pinctrl_detach),
    DEVMETHOD(fdt_pinctrl_configure, mt7622_pinctrl_configure),
    DEVMETHOD_END
};

static DEFINE_CLASS_0(pinmux, mt7622_pinmux_driver, mt7622_pinmux_methods,
    sizeof(struct mt7622_pinctrl_softc));
EARLY_DRIVER_MODULE(mt7622_pinmux, simplebus, mt7622_pinmux_driver, NULL, NULL, 71);
