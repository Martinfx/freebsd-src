/*
 * memset using MOPS extension.
 *
 * Copyright (c) 2023, Arm Limited.
 * SPDX-License-Identifier: MIT OR Apache-2.0 WITH LLVM-exception
 */

#include "asmdefs.h"

ENTRY (__memset_aarch64_mops)
	mov     x3, x0
	.inst   0x19c10443	/* setp    [x3]!, x2!, x1  */
	.inst   0x19c14443	/* setm    [x3]!, x2!, x1  */
	.inst   0x19c18443	/* sete    [x3]!, x2!, x1  */
	ret

END (__memset_aarch64_mops)
