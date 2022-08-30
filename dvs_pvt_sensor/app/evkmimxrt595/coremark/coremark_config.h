/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _COREMARK_CONIFG_
#define _COREMARK_CONFIG_

/* Configures coremark to run ~10 seconds for both debug and release build configs */
#if defined(NDEBUG)
#define ITERATIONS 6000
#define COMPILER_FLAGS "-O3"
#else
#define ITERATIONS 1300
#define COMPILER_FLAGS "-O0"
#endif

#endif /* _COREMARK_CONFIG_ */