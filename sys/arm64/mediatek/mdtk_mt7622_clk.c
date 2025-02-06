#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <machine/bus.h>
#include <dev/fdt/simplebus.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>

#include <dt-bindings/clock/mt7622-clk.h>

#include <arm64/mediatek/mdtk_clk.h>

#define mdtk_mt7622_CLK_BASE        0x10210000
#define CLK_CFG_0              (mdtk_mt7622_CLK_BASE + 0x40)
#define CLK_CFG_0_SET          (mdtk_mt7622_CLK_BASE + 0x44)
#define CLK_CFG_0_CLR          (mdtk_mt7622_CLK_BASE + 0x48)
#define CLK_CFG_1              (mdtk_mt7622_CLK_BASE + 0x50)


static int
mdtk_mt7622_clk_probe(device_t dev)
{

	if (!ofw_bus_status_okay(dev))
		return (ENXIO);

	if (ofw_bus_is_compatible(dev, "mediatek,mt7622-clk")) {
		device_set_desc(dev, "Mediatek Unit mt7622-clk");
		return (BUS_PROBE_DEFAULT);
	}

	return (ENXIO);
}

static int
mdtk_mt7622_clk_attach(device_t dev) {
	struct mdtk_clk_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;

	//sc->gates = mdtk_mt7622_gates;
	//sc->ngates = nitems(mdtk_mt7622_gates);

	//sc->clks = mdtk_mt7622_clks;
	//sc->nclks = nitems(mdtk_mt7622_clks);

	//sc->reset_num = CRU_SOFTRST_SIZE * 16;
	//sc->reset_offset = CRU_SOFTRST_CON(0);

	return (mdtk_attach(dev));
}

static device_method_t mdtk_mt7622_clk_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		mdtk_mt7622_clk_probe),
	DEVMETHOD(device_attach,	mdtk_mt7622_clk_attach),
	DEVMETHOD_END
};


DEFINE_CLASS_1(mdtk_mt7622_clk, mdtk_mt7622_clk_driver, mdtk_mt7622_clk_methods,
  sizeof(struct mdtk_clk_softc), mdtk_clk_driver);

EARLY_DRIVER_MODULE(mdtk_mt7622_clk, simplebus, mdtk_mt7622_clk_driver, 0, 0,
    BUS_PASS_BUS + BUS_PASS_ORDER_MIDDLE + 1);
