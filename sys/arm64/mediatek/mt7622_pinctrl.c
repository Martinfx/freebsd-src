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

struct pinmux_softc {
    device_t 	dev;
    struct resource	*base_res;
    struct resource	*eint_res;
};

static struct ofw_compat_data compat_data[] = {
    {"mediatek,mt7622-pinctrl",	1},
    {NULL,				0},
};

/* Numeric based parameters. */
/*
static const struct prop_name {
    const char *name;
   // enum prop_id id;
} prop_names[] = {

};*/

/*
 * configuration for one pin group.
 */
struct pincfg {
    char	*function;
    //int	params[PROP_ID_MAX_ID];
};

struct mt_grp {
    char *name;
    bus_size_t reg;
    int drvdn_shift;
    int drvdn_mask;
    int drvup_shift;
    int drvup_mask;
};


struct mt_mux {
    struct mt_grp grp;
    char *name;
    bus_size_t reg;
    char *functions[4];
    int gpio_num;

};

/*
static const struct mt_grp *
pinmux_search_grp(char *grp_name)
{
    int i;

    for (i = 0; i < nitems(pin_grp_tbl); i++) {
        if (strcmp(grp_name, pin_grp_tbl[i].name) == 0)
            return 	(&pin_grp_tbl[i]);
    }
    return (NULL);
}

static const struct mt_mux *
pinmux_search_mux(char *pin_name)
{
    int i;

    for (i = 0; i < nitems(pin_mux_tbl); i++) {
        if (strcmp(pin_name, pin_mux_tbl[i].name) == 0)
            return 	(&pin_mux_tbl[i]);
    }
    return (NULL);
}

static int
pinmux_mux_function(const struct mt_mux *mux, char *fnc_name)
{
    int i;

    for (i = 0; i < 4; i++) {
        if (strcmp(fnc_name, mux->functions[i]) == 0)
            return 	(i);
    }
    return (-1);
}*/
/*
static int
pinmux_config_mux(struct pinmux_softc *sc, char *pin_name,
    const struct tegra_mux *mux, struct pincfg *cfg)
{
    int tmp;
    uint32_t reg;

    reg = bus_read_4(sc->mux_mem_res, mux->reg);

    if (cfg->function != NULL) {
        tmp = pinmux_mux_function(mux, cfg->function);
        if (tmp == -1) {
            device_printf(sc->dev,
                "Unknown function %s for pin %s\n", cfg->function,
                pin_name);
            return (ENXIO);
        }
        reg &= ~(TEGRA_MUX_FUNCTION_MASK << TEGRA_MUX_FUNCTION_SHIFT);
        reg |=  (tmp & TEGRA_MUX_FUNCTION_MASK) <<
            TEGRA_MUX_FUNCTION_SHIFT;
    }
    if (cfg->params[PROP_ID_PULL] != -1) {
        reg &= ~(TEGRA_MUX_PUPD_MASK << TEGRA_MUX_PUPD_SHIFT);
        reg |=  (cfg->params[PROP_ID_PULL] & TEGRA_MUX_PUPD_MASK) <<
            TEGRA_MUX_PUPD_SHIFT;
    }
    if (cfg->params[PROP_ID_TRISTATE] != -1) {
        reg &= ~(1 << TEGRA_MUX_TRISTATE_SHIFT);
        reg |=  (cfg->params[PROP_ID_TRISTATE] & 1) <<
            TEGRA_MUX_TRISTATE_SHIFT;
    }
    if (cfg->params[TEGRA_MUX_ENABLE_INPUT_SHIFT] != -1) {
        reg &= ~(1 << TEGRA_MUX_ENABLE_INPUT_SHIFT);
        reg |=  (cfg->params[TEGRA_MUX_ENABLE_INPUT_SHIFT] & 1) <<
            TEGRA_MUX_ENABLE_INPUT_SHIFT;
    }
    if (cfg->params[PROP_ID_ENABLE_INPUT] != -1) {
        reg &= ~(1 << TEGRA_MUX_ENABLE_INPUT_SHIFT);
        reg |=  (cfg->params[PROP_ID_ENABLE_INPUT] & 1) <<
            TEGRA_MUX_ENABLE_INPUT_SHIFT;
    }
    if (cfg->params[PROP_ID_ENABLE_INPUT] != -1) {
        reg &= ~(1 << TEGRA_MUX_ENABLE_INPUT_SHIFT);
        reg |=  (cfg->params[PROP_ID_OPEN_DRAIN] & 1) <<
            TEGRA_MUX_ENABLE_INPUT_SHIFT;
    }
    if (cfg->params[PROP_ID_LOCK] != -1) {
        reg &= ~(1 << TEGRA_MUX_LOCK_SHIFT);
        reg |=  (cfg->params[PROP_ID_LOCK] & 1) <<
            TEGRA_MUX_LOCK_SHIFT;
    }
    if (cfg->params[PROP_ID_IORESET] != -1) {
        reg &= ~(1 << TEGRA_MUX_IORESET_SHIFT);
        reg |=  (cfg->params[PROP_ID_IORESET] & 1) <<
            TEGRA_MUX_IORESET_SHIFT;
    }
    if (cfg->params[PROP_ID_RCV_SEL] != -1) {
        reg &= ~(1 << TEGRA_MUX_RCV_SEL_SHIFT);
        reg |=  (cfg->params[PROP_ID_RCV_SEL] & 1) <<
            TEGRA_MUX_RCV_SEL_SHIFT;
    }
    bus_write_4(sc->mux_mem_res, mux->reg, reg);
    return (0);
}

static int
pinmux_config_grp(struct pinmux_softc *sc, char *grp_name,
    const struct tegra_grp *grp, struct pincfg *cfg)
{
    uint32_t reg;

    reg = bus_read_4(sc->pad_mem_res, grp->reg);

    if (cfg->params[PROP_ID_HIGH_SPEED_MODE] != -1) {
        reg &= ~(1 << TEGRA_GRP_HSM_SHIFT);
        reg |=  (cfg->params[PROP_ID_HIGH_SPEED_MODE] & 1) <<
            TEGRA_GRP_HSM_SHIFT;
    }
    if (cfg->params[PROP_ID_SCHMITT] != -1) {
        reg &= ~(1 << TEGRA_GRP_SCHMT_SHIFT);
        reg |=  (cfg->params[PROP_ID_SCHMITT] & 1) <<
            TEGRA_GRP_SCHMT_SHIFT;
    }
    if (cfg->params[PROP_ID_DRIVE_TYPE] != -1) {
        reg &= ~(TEGRA_GRP_DRV_TYPE_MASK << TEGRA_GRP_DRV_TYPE_SHIFT);
        reg |=  (cfg->params[PROP_ID_DRIVE_TYPE] &
            TEGRA_GRP_DRV_TYPE_MASK) << TEGRA_GRP_DRV_TYPE_SHIFT;
    }
    if (cfg->params[PROP_ID_SLEW_RATE_RISING] != -1) {
        reg &= ~(TEGRA_GRP_DRV_DRVDN_SLWR_MASK <<
            TEGRA_GRP_DRV_DRVDN_SLWR_SHIFT);
        reg |=  (cfg->params[PROP_ID_SLEW_RATE_RISING] &
            TEGRA_GRP_DRV_DRVDN_SLWR_MASK) <<
            TEGRA_GRP_DRV_DRVDN_SLWR_SHIFT;
    }
    if (cfg->params[PROP_ID_SLEW_RATE_FALLING] != -1) {
        reg &= ~(TEGRA_GRP_DRV_DRVUP_SLWF_MASK <<
            TEGRA_GRP_DRV_DRVUP_SLWF_SHIFT);
        reg |=  (cfg->params[PROP_ID_SLEW_RATE_FALLING] &
            TEGRA_GRP_DRV_DRVUP_SLWF_MASK) <<
            TEGRA_GRP_DRV_DRVUP_SLWF_SHIFT;
    }
    if ((cfg->params[PROP_ID_DRIVE_DOWN_STRENGTH] != -1) &&
         (grp->drvdn_mask != 0)) {
        reg &= ~(grp->drvdn_shift << grp->drvdn_mask);
        reg |=  (cfg->params[PROP_ID_DRIVE_DOWN_STRENGTH] &
            grp->drvdn_mask) << grp->drvdn_shift;
    }
    if ((cfg->params[PROP_ID_DRIVE_UP_STRENGTH] != -1) &&
         (grp->drvup_mask != 0)) {
        reg &= ~(grp->drvup_shift << grp->drvup_mask);
        reg |=  (cfg->params[PROP_ID_DRIVE_UP_STRENGTH] &
            grp->drvup_mask) << grp->drvup_shift;
    }
    bus_write_4(sc->pad_mem_res, grp->reg, reg);
    return (0);
}
*/
/*
static int
pinmux_config_node(struct pinmux_softc *sc, char *pin_name, struct pincfg *cfg)
{
    const struct mt_mux *mux;
    const struct mt_grp *grp;
    bool handled;
    int rv;

    mux = pinmux_search_mux(pin_name);
    handled = false;
    if (mux != NULL) {
        if (mux->gpio_num != -1) {

        }
        rv = pinmux_config_mux(sc, pin_name, mux, cfg);
        if (rv != 0)
            return (rv);
        if (mux->grp.reg <= 0) {
            rv = pinmux_config_grp(sc, pin_name, &mux->grp, cfg);
            return (rv);
        }
        handled = true;
    }


    grp = pinmux_search_grp(pin_name);
    if (grp != NULL) {
        rv = pinmux_config_grp(sc, pin_name, grp, cfg);
        if (rv != 0)
            return (rv);
        handled = true;
    }

    if (!handled) {
        device_printf(sc->dev, "Unknown pin: %s\n", pin_name);
        return (ENXIO);
    }
    return (0);
}
*/
/*static int
pinmux_read_node(struct pinmux_softc *sc, phandle_t node, struct pincfg *cfg,
    char **pins, int *lpins)
{
    int rv, i;

    *lpins = OF_getprop_alloc(node, "nvidia,pins", (void **)pins);
    if (*lpins <= 0)
        return (ENOENT);

    rv = OF_getprop_alloc(node, "nvidia,function", (void **)&cfg->function);
    if (rv <= 0)
        cfg->function = NULL;

    for (i = 0; i < PROP_ID_MAX_ID; i++) {
        rv = OF_getencprop(node, prop_names[i].name, &cfg->params[i],
            sizeof(cfg->params[i]));
        if (rv <= 0)
            cfg->params[i] = -1;
    }
    return (0);
}

static int
pinmux_process_node(struct pinmux_softc *sc, phandle_t node)
{
    struct pincfg cfg;
    char *pins, *pname;
    int i, len, lpins, rv;

    rv = pinmux_read_node(sc, node, &cfg, &pins, &lpins);
    if (rv != 0)
        return (rv);

    len = 0;
    pname = pins;
    do {
        i = strlen(pname) + 1;
        rv = pinmux_config_node(sc, pname, &cfg);
        if (rv != 0)
            device_printf(sc->dev, "Cannot configure pin: %s: %d\n",
                pname, rv);
        len += i;
        pname += i;
    } while (len < lpins);

    if (pins != NULL)
        OF_prop_free(pins);
    if (cfg.function != NULL)
        OF_prop_free(cfg.function);
    return (rv);
}
*/
static int pinmux_configure(device_t dev, phandle_t cfgxref)
{
    //struct pinmux_softc *sc;
    phandle_t node, cfgnode;

    //sc = device_get_softc(dev);
    cfgnode = OF_node_from_xref(cfgxref);


    for (node = OF_child(cfgnode); node != 0; node = OF_peer(node)) {
        if (!ofw_bus_node_status_okay(node))
            continue;
       // pinmux_process_node(sc, node);
    }
    return (0);
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
    struct pinmux_softc * sc;
    int rid;

    sc = device_get_softc(dev);
    sc->dev = dev;

    rid = 0;
    sc->base_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
        RF_ACTIVE);
    if (sc->base_res == NULL) {
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


    /* Register as a pinctrl device and process default configuration */
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
    sizeof(struct pinmux_softc));
EARLY_DRIVER_MODULE(mt7622_pinmux, simplebus, mt7622_pinmux_driver,
    NULL, NULL, 71);
