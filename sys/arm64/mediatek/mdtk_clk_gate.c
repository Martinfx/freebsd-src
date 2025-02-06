#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <machine/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <dev/clk/clk_gate.h>
#include <dev/clk/clk.h>

#include <arm64/mediatek/mdtk_clk_gate.h>

#define	WR4(_clk, off, val)						\
	CLKDEV_WRITE_4(clknode_get_device(_clk), off, val)
#define	RD4(_clk, off, val)						\
	CLKDEV_READ_4(clknode_get_device(_clk), off, val)
#define	MD4(_clk, off, clr, set )					\
	CLKDEV_MODIFY_4(clknode_get_device(_clk), off, clr, set)
#define	DEVICE_LOCK(_clk)						\
	CLKDEV_DEVICE_LOCK(clknode_get_device(_clk))
#define	DEVICE_UNLOCK(_clk)						\
	CLKDEV_DEVICE_UNLOCK(clknode_get_device(_clk))

static int mdtk_clk_gate_init(struct clknode *clk, device_t dev);
//static int mdtk_clk_gate_get_gate(struct clknode *clk, bool *enabled);
static int mdtk_clk_gate_set_gate(struct clknode *clk, bool enable);

struct mdtk_clk_gate_sc {
	struct clknode *clk;
	device_t dev;
	struct resource *res;
	uint32_t set_ofs;
	uint32_t clr_ofs;
	uint32_t sta_ofs;
	uint8_t bit;
	bool ungated;
};

static clknode_method_t mdtk_clk_gate_methods[] = {
	/* Device interface */
	CLKNODEMETHOD(clknode_init,	mdtk_clk_gate_init),
	CLKNODEMETHOD(clknode_set_gate,	mdtk_clk_gate_set_gate),
	CLKNODEMETHOD_END
};
DEFINE_CLASS_1(mdtk_clk_gate, mdtk_clk_gate_class, mdtk_clk_gate_methods,
   sizeof(struct mdtk_clk_gate_sc), clknode_class);
/*
static int
mdtk_get_clockgating(struct clknode *clk, bool inverse)
{
    struct mdtk_clk_gate_sc *sc;
    uint32_t val;

    sc = clknode_get_softc(clk);
    val = bus_read_4(sc->res, sc->sta_ofs);

    if (inverse)
        return !(val & (1 << sc->bit));
    else
        return (val & (1 << sc->bit));
}

static void
mdtk_cg_set_bit(struct clknode *clk)
{
    struct mdtk_clk_gate_sc *sc = clknode_get_softc(clk);
    bus_write_4(sc->res, sc->set_ofs, (1 << sc->bit));
}

static void
mdtk_cg_clr_bit(struct clknode *clk)
{
    struct mdtk_clk_gate_sc *sc = clknode_get_softc(clk);
    bus_write_4(sc->res, sc->clr_ofs, (1 << sc->bit));
}

static int
mdtk_clk_gate_enable(struct clknode *clk)
{
    mdtk_cg_clr_bit(clk);
    return (0);
}

static int
mdtk_clk_gate_disable(struct clknode *clk)
{
    mdtk_cg_set_bit(clk);
    return (0);
}

static int
mdtk_clk_gate_is_enabled(struct clknode *clk)
{
    return mdtk_get_clockgating(clk, false);
}
*/
static int
mdtk_clk_gate_init(struct clknode *clk, device_t dev)
{
	uint32_t reg = 0;
	struct mdtk_clk_gate_sc *sc;
	int rv;

	sc = clknode_get_softc(clk);
	sc->dev = dev;

	DEVICE_LOCK(clk);
	rv = bus_read_4(sc->res, sc->sta_ofs);
	DEVICE_UNLOCK(clk);

	if (rv < 0)
		return (rv);

	sc->ungated = (reg & (1 << sc->bit)) ? true : false;
	clknode_init_parent_idx(clk, 0);
	return (0);
}

static int
mdtk_clk_gate_set_gate(struct clknode *clk, bool enable)
{
	struct mdtk_clk_gate_sc *sc;
	//uint32_t reg = 0;

	sc = clknode_get_softc(clk);
	sc->ungated = enable;

	DEVICE_LOCK(clk);
	if (enable)
		bus_write_4(sc->res, sc->clr_ofs, (1 << sc->bit));
	else
		bus_write_4(sc->res, sc->set_ofs, (1 << sc->bit));

	//reg = bus_read_4(sc->res, sc->sta_ofs);
	DEVICE_UNLOCK(clk);

	return (0);
}
/*
static int
mdtk_clk_gate_get_gate(struct clknode *clk, bool *enabled)
{
	struct mdtk_clk_gate_sc *sc;
	uint32_t reg;

	sc = clknode_get_softc(clk);

	DEVICE_LOCK(clk);
	reg = bus_read_4(sc->res, sc->sta_ofs);
	DEVICE_UNLOCK(clk);

	*enabled = (reg & (1 << sc->bit)) ? true : false;
	return (0);
}
*/
int
mdtk_clk_gate_register(struct clkdom *clkdom, struct mdtk_clk_gate_def *clkdef)
{
	struct clknode *clk;
	struct mdtk_clk_gate_sc *sc;

	clk = clknode_create(clkdom, &mdtk_clk_gate_class, &clkdef->clkdef);
	if (clk == NULL)
		return (1);

	sc = clknode_get_softc(clk);
	sc->res = clkdef->res;
	sc->set_ofs = clkdef->set_ofs;
	sc->clr_ofs = clkdef->clr_ofs;
	sc->sta_ofs = clkdef->sta_ofs;
	sc->bit = clkdef->bit;

	clknode_register(clkdom, clk);
	return (0);
}



