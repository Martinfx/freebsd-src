#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <machine/bus.h>

#include <dev/fdt/simplebus.h>

#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dev/clk/clk.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk_fixed.h>
#include <dev/clk/clk_link.h>

#include "clkdev_if.h"
#include "hwreset_if.h"

#include <arm64/mediatek/mdtk_clk.h>
#include <arm64/mediatek/mdtk_clk_gate.h>

static struct resource_spec mdtk_spec[] = {
    { SYS_RES_MEMORY,	0,	RF_ACTIVE },
    { -1, 0 }
};

#define	CCU_READ4(sc, reg)		bus_read_4((sc)->res, (reg))
#define	CCU_WRITE4(sc, reg, val)	bus_write_4((sc)->res, (reg), (val))

static int
mdtk_write_4(device_t dev, bus_addr_t addr, uint32_t val)
{
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    CCU_WRITE4(sc, addr, val);
    return (0);
}

static int
mdtk_read_4(device_t dev, bus_addr_t addr, uint32_t *val)
{
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);

    *val = CCU_READ4(sc, addr);
    return (0);
}

static int
mdtk_modify_4(device_t dev, bus_addr_t addr, uint32_t clr, uint32_t set)
{
    struct mdtk_clk_softc *sc;
    uint32_t reg;

    sc = device_get_softc(dev);

    reg = CCU_READ4(sc, addr);
    reg &= ~clr;
    reg |= set;
    CCU_WRITE4(sc, addr, reg);

    return (0);
}

static int
mdtk_reset_assert(device_t dev, intptr_t id, bool reset)
{
    struct mdtk_clk_softc *sc;
    uint32_t reg;
    int bit;
    uint32_t val;

    sc = device_get_softc(dev);

    if (id > sc->reset_num)
        return (ENXIO);

    reg = sc->reset_offset + id / 16 * 4;
    bit = id % 16;

    mtx_lock(&sc->mtx);
    val = 0;
    if (reset)
        val = (1 << bit);
    CCU_WRITE4(sc, reg, val | ((1 << bit) << 16));
    mtx_unlock(&sc->mtx);

    return (0);
}

static int
mdtk_reset_is_asserted(device_t dev, intptr_t id, bool *reset)
{
    struct mdtk_clk_softc *sc;
    uint32_t reg;
    int bit;
    uint32_t val;

    sc = device_get_softc(dev);

    if (id > sc->reset_num)
        return (ENXIO);
    reg = sc->reset_offset + id / 16 * 4;
    bit = id % 16;

    mtx_lock(&sc->mtx);
    val = CCU_READ4(sc, reg);
    mtx_unlock(&sc->mtx);

    *reset = false;
    if (val & (1 << bit))
        *reset = true;

    return (0);
}

static void
mdtk_device_lock(device_t dev)
{
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    mtx_lock(&sc->mtx);
}

static void
mdtk_device_unlock(device_t dev)
{
    struct mdtk_clk_softc *sc;

    sc = device_get_softc(dev);
    mtx_unlock(&sc->mtx);
}

static int
mdtk_register_gates(struct mdtk_clk_softc *sc)
{
    struct mdtk_clk_gate_def def;
    int i;

    for (i = 0; i < sc->ngates; i++) {
        if (sc->gates[i].name == NULL)
            continue;
        memset(&def, 0, sizeof(def));
        def.clkdef.id = sc->gates[i].id;
        def.clkdef.name = sc->gates[i].name;
        def.clkdef.parent_names = &sc->gates[i].parent_name;
        def.clkdef.parent_cnt = 1;
        def.offset = sc->gates[i].offset;
        def.shift = sc->gates[i].shift;
        def.mask = 1;
        def.on_value = 0;
        def.off_value = 1;
        mdtk_clk_gate_register(sc->clkdom, &def);
    }

    return (0);
}

int
mdtk_attach(device_t dev)
{
	struct mdtk_clk_softc *sc;
	phandle_t node;
	int	i;

	sc = device_get_softc(dev);
	sc->dev = dev;

	node = ofw_bus_get_node(dev);

	if (bus_alloc_resources(dev, mdtk_spec, &sc->res) != 0) {
		device_printf(dev, "cannot allocate resources for device\n");
		return (ENXIO);
	}

	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	sc->clkdom = clkdom_create(dev);
	if (sc->clkdom == NULL)
		panic("Cannot create clkdom\n");

	for (i = 0; i < sc->nclks; i++) {
		switch (sc->clks[i].type) {
		case MK_CLK_UNDEFINED:
			break;
		/*case RK3066_CLK_PLL:
			rk3066_clk_pll_register(sc->clkdom,
			    sc->clks[i].clk.pll);
			break;
		case RK3328_CLK_PLL:
			rk3328_clk_pll_register(sc->clkdom,
			    sc->clks[i].clk.pll);
			break;
		case RK3399_CLK_PLL:
			rk3399_clk_pll_register(sc->clkdom,
			    sc->clks[i].clk.pll);
			break;
		case RK_CLK_COMPOSITE:
			rk_clk_composite_register(sc->clkdom,
			    sc->clks[i].clk.composite);
			break;
		case RK_CLK_MUX:
			rk_clk_mux_register(sc->clkdom, sc->clks[i].clk.mux);
			break;
		case RK_CLK_ARMCLK:
			rk_clk_armclk_register(sc->clkdom,
			    sc->clks[i].clk.armclk);
			break;
		case RK_CLK_FIXED:
			clknode_fixed_register(sc->clkdom,
			    sc->clks[i].clk.fixed);
			break;
		case RK_CLK_FRACT:
			rk_clk_fract_register(sc->clkdom,
			    sc->clks[i].clk.fract);
			break;
		case RK_CLK_LINK:
			clknode_link_register(sc->clkdom,
			    sc->clks[i].clk.link);
			break;*/
		default:
			device_printf(dev, "Unknown clock type\n");
			return (ENXIO);
		}
	}

	if (sc->gates)
		mdtk_register_gates(sc);

	if (clkdom_finit(sc->clkdom) != 0)
		panic("cannot finalize clkdom initialization\n");

	if (bootverbose)
		clkdom_dump(sc->clkdom);

	clk_set_assigned(dev, node);

	/* register our self as a reset provider */
	//hwreset_register_ofw_provider(dev);

	return (0);
}

static device_method_t mdtk_methods[] = {
	/* clkdev interface */
	DEVMETHOD(clkdev_write_4,	mdtk_write_4),
	DEVMETHOD(clkdev_read_4,	mdtk_read_4),
	DEVMETHOD(clkdev_modify_4,	mdtk_modify_4),
	DEVMETHOD(clkdev_device_lock,	mdtk_device_lock),
	DEVMETHOD(clkdev_device_unlock,	mdtk_device_unlock),

	/* Reset interface */
	//DEVMETHOD(hwreset_assert,	mdtk_reset_assert),
	//DEVMETHOD(hwreset_is_asserted,	mdtk_reset_is_asserted),

	DEVMETHOD_END
};

DEFINE_CLASS_0(mdtk_clk, mdtk_clk_driver, mdtk_methods,
    sizeof(struct mdtk_clk_softc));
