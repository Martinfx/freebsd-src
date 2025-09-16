#ifndef SOPHGO_CLK_H
#define SOPHGO_CLK_H

/*-
* SPDX-License-Identifier: BSD-2-Clause
*
* Copyright (c) 2025 Martin Filla
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
* THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
* OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
* AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
* OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
* SUCH DAMAGE.
 */
/* Parent list */
#define PLIST(_name) static const char *_name[]

struct sophgo_clk_def {
	struct clk_pll_def *pll_def;
	struct clk_link_def *linked_def;
	struct clk_fixed_def *fixed_def;
	struct clk_mux_def *muxes_def;
	struct clk_gate_def *gates_def;
	struct clk_div_def *dived_def;
	int num_pll;
	int num_linked;
	int num_fixed;
	int num_muxes;
	int num_gates;
	int num_dived;
};

struct sophgo_clk_softc {
	device_t dev;
	struct resource *mem_res;
	struct mtx mtx;
	struct clkdom *clkdom;
	struct syscon *syscon;
};

int sophgo_clkdev_read_4(device_t dev, bus_addr_t addr, uint32_t *val);
int sophgo_clkdev_write_4(device_t dev, bus_addr_t addr, uint32_t val);
int sophgo_clkdev_modify_4(device_t dev, bus_addr_t addr, uint32_t clear_mask,
    uint32_t set_mask);

void sophgo_clkdev_device_lock(device_t dev);
void sophgo_clkdev_device_unlock(device_t dev);
void sophgo_register_clocks(device_t dev, struct sophgo_clk_def *cldef);
int sophgo_hwreset_by_idx(struct sophgo_clk_softc *sc, intptr_t idx, bool reset);

#endif