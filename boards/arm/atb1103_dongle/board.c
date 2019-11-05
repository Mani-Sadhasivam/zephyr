/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief board init functions
 */

#include <init.h>
#include "board.h"

static const struct acts_pin_config board_pin_config[] = {
	/* uart0 */
	{2, 3 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3)},
	{3, 3 | GPIO_CTL_SMIT | GPIO_CTL_PADDRV_LEVEL(3)},
};

static int board_early_init(struct device *arg)
{
	ARG_UNUSED(arg);

	/* remap vector */
	sys_write32(0x01000000, VECTOR_BASE);
	sys_write32(sys_read32(MEM_CTL) | MEM_CTL_VECTOR_TABLE_SEL, MEM_CTL);
	
	acts_pinmux_setup_pins(board_pin_config, ARRAY_SIZE(board_pin_config));

	/* set vd12_ok for RC3M */
	sys_write32(sys_read32(COREPLL_DBGCTL) | (0x1<<12), COREPLL_DBGCTL);

	return 0;
}

SYS_INIT(board_early_init, PRE_KERNEL_1, 1);
