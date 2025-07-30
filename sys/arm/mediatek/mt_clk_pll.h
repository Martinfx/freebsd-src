/*
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Martin Filla
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

#ifndef __MT_CLK_PLL_H__
#define __MT_CLK_PLL_H__

struct div_table {
    uint32_t div;
    uint64_t freq;
};

struct clk_pll_def {
    struct clknode_init_def clkdef;
    bus_size_t pll_base_reg;
    bus_size_t pll_pwr_reg;
    uint32_t pll_flags;
    uint64_t pll_fmin;
    uint64_t pll_fmax;
    uint32_t pll_en_mask;
    bus_size_t pll_pd_reg;
    bus_size_t pll_tuner_reg;
    uint32_t pll_tuner_en_bit;
    int pll_pd_shift;
    int pll_pcwbits;
    int pll_pcwibits;
    bus_size_t pll_pcw_reg;
    uint32_t pll_pcw_chg_reg;
    int pll_pcw_shift;
    struct div_table *pll_div_table;
    uint32_t pll_rst_bar_mask;
};

int mt_clk_pll_register(struct clkdom *clkdom,
                        struct clk_pll_def *clkdef);

#endif // __MT_CLK_PLL_H__
