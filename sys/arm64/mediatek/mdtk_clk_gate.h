#ifndef MDTK_CLK_GATE_H
#define MDTK_CLK_GATE_H

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <dev/clk/clk.h>
#include "clkdev_if.h"

struct mdtk_clk_gate_def {
	struct clknode_init_def clkdef;
	struct resource *res;
	uint32_t set_ofs;
	uint32_t clr_ofs;
	uint32_t sta_ofs;
	uint8_t bit;
    uint32_t offset;          
    uint8_t shift;            
    uint8_t mask;             
    uint8_t on_value;         
    uint8_t off_value;        
};

#define MDTK_GATE(_name, _parent, _regs, _shift, _flags) { \
        .name = _name, \
        .parent_name = _parent, \
        .regs = _regs, \
        .shift = _shift, \
        .flags = _flags, \
}


int mdtk_clk_gate_register(struct clkdom *clkdom, struct mdtk_clk_gate_def *clkdef);

#endif
