/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief System/hardware module
 *
 * This module provides routines to initialize and support board-level hardware.
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include "soc.h"

int acts_init(struct device *arg)
{
	ARG_UNUSED(arg);

	/* init vd12 */
	acts_request_vd12_pd(false);
	acts_request_vd12_largebias(false);

	/* disable rc_3M */
	acts_request_rc_3M(false);

	return 0;
}

SYS_INIT(acts_init, PRE_KERNEL_1, 0);
