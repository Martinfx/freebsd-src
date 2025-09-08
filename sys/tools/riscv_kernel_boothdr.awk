#!/usr/bin/awk -f
#-
# SPDX-License-Identifier: BSD-2-Clause
#
# Copyright 2025 Martin Filla
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
# OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
# SUCH DAMAGE.
#

BEGIN {
    # Init global vars.
    gBytesOut = 0;  # How many output bytes we've written so far
    gKernbase = 0;  # Address of first byte of loaded kernel image
    gStart = 0;     # Address of _start symbol
    gStartOff = 0;  # Offset of _start symbol from start of image
    gEnd = 0;       # Address of _end symbol
    gEndOff = 0;    # Offset of _end symbol from start of image

    # The type of header we're writing is set using -v hdrtype= on
    # the command line, ensure we got a valid value for it.
    if (hdrtype != "rvbooti") {
        print "riscv_kernel_boothdr.awk: " \
		    "missing or invalid '-v hdrtype=' argument" >"/dev/stderr"
        gHdrType = "error_reported"
        exit 1
    }

    gHdrType = hdrtype
    for (i = 0; i < 16; i++) {
        hex[sprintf("%x", i)] = i;
        hex[sprintf("%X", i)] = i;
    }
}

function addr_to_offset(addr) {
    # Turn an address into an offset from the start of the loaded image.
    return addr % gKernbase
}

function hexstr_to_num(str) {

    sum = 0;
    len = length(str);
    for (i = 1; i <= len; i++) {
        sum = sum * 16 + hex[substr(str, i, 1)];
    }

    return sum;
}

function write_le32(num) {

    for (i = 0; i < 4; i++) {
        printf("%c", num % 256);
        num /= 256
    }
    gBytesOut += 4
}

function write_le64(num) {

    for (i = 0; i < 8; i++) {
        printf("%c", num % 256);
        num /= 256
    }
    gBytesOut += 8
}

function write_padding() {

    # Write enough padding bytes so that the header fills all the
    # remaining space before the _start symbol.

    while (gBytesOut++ < gStartOff) {
        printf("%c", 0);
    }
}

function write_rvjump() {
    # We are writing a JAL instruction to jump to _start.
    imm = rshift(gStartOff + 64, 1)
    inst = 0
    imm_20    = and(rshift(imm, 20), 1)
    imm_10_1  = and(rshift(imm,  1), hexstr_to_num("3FF"))
    imm_11    = and(rshift(imm, 11), 1)
    imm_19_12 = and(rshift(imm, 12), hexstr_to_num("FF"))

    inst = or(inst, lshift(imm_20,    31))
    inst = or(inst, lshift(imm_10_1,  21))
    inst = or(inst, lshift(imm_11,    20))
    inst = or(inst, lshift(imm_19_12, 12))
    inst = or(inst, lshift(0, 7))                 # rd = x0
    inst = or(inst, hexstr_to_num("6F"))          # opcode JAL
    write_le32(inst)
}

function write_rvbooti() {

    # We are writing this struct...
    # u32 code0;                /* Executable code */
    # u32 code1;                /* Executable code */
    # u64 text_offset;          /* Image load offset, little endian */
    # u64 image_size;           /* Effective Image size, little endian */
    # u64 flags;                /* kernel flags, little endian */
    # u32 version;              /* Version of this header */
    # u32 res1 = 0;             /* Reserved */
    # u64 res2 = 0;             /* Reserved */
    # u64 magic = 0x5643534952; /* Magic number, little endian, "RISCV" */
    # u32 magic2 = 0x05435352;  /* Magic number 2, little endian, "RSC\x05" */
    # u32 res3;                 /* Reserved for PE COFF offset */

    write_rvjump()                           # code0
    write_le32(0)                            # code1: reserved (Linux 0)
    write_le64(0)                            # text_offset (0 => auto)
    write_le64(gEndOff)                      # image_size (od 0 po _end)
    write_le64(0)                            # flags (0 = LE)
    write_le32(hexstr_to_num("00000002"))  # version (v0.2 => minor=2)
    write_le32(0)                            # res1
    write_le64(0)                            # res2
    write_le64(hexstr_to_num("5643534952"))# "RISCV" (LE)
    write_le32(hexstr_to_num("05435352"))  # "RSC\x05" (LE)
    write_le32(0)                            # res3 (PE/COFF offset placeholder)
}


/kernbase/ {
    # If the symbol name is exactly "kernbase" save its address.
    if ($3 == "kernbase") {
        gKernbase = hexstr_to_num($1)
    }
}

/_start/ {
    # If the symbol name is exactly "_start" save its address.
    if ($3 == "_start") {
        gStart = hexstr_to_num($1)
    }
}

/_end/ {
    # If the symbol name is exactly "_end" remember its value.
    if ($3 == "_end") {
        gEnd = hexstr_to_num($1)
    }
}

END {
    # Note that this function runs even if BEGIN calls exit(1)!
    if (gHdrType == "error_reported") {
        exit 1
    }

    # Make sure we got all three required symbols.
    if (gKernbase == 0 || gStart == 0 || gEnd == 0) {
        print "riscv_kernel_boothdr.awk: " \
		    "missing kernbase/_start/_end symbol(s)" >"/dev/stderr"
        exit 1
    }

    gStartOff = addr_to_offset(gStart)
    gEndOff = addr_to_offset(gEnd)

    if (gHdrType == "rvbooti") {
        write_rvbooti()
    }
    write_padding()
}