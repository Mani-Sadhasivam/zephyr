/*
 * Copyright (c) 2013-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Reset handler
 *
 * Reset handler that prepares the system for running C code.
 */

#include <board.h>
#include <toolchain.h>
#include <linker/sections.h>
#include <arch/cpu.h>
#include <offsets_short.h>

_ASM_FILE_PROLOGUE

#if defined(CONFIG_PLATFORM_SPECIFIC_INIT)
GTEXT(_PlatformInit)
SECTION_FUNC(TEXT,_PlatformInit)
setup_sp:
    /* setup sp */
    ldr r0, =0x20009f00
    mov sp, r0

    bx      lr

#endif
