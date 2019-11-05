/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file peripheral reset interface for Actions SoC
 */

#include <kernel.h>
#include "soc.h"

static void acts_reset_peripheral_control(int reset_id, int assert)
{
	uint32_t key;

	if (reset_id > RESET_ID_MAX_ID)
		return;

	key = irq_lock();

	if (assert)
		sys_write32(sys_read32(CMU_DEVRST_REG) & ~(1 << reset_id),
			    CMU_DEVRST_REG);
	else
		sys_write32(sys_read32(CMU_DEVRST_REG) | (1 << reset_id),
			    CMU_DEVRST_REG);

	irq_unlock(key);
}

void acts_reset_peripheral_assert(int reset_id)
{
	acts_reset_peripheral_control(reset_id, 1);
}

void acts_reset_peripheral_deassert(int reset_id)
{
	acts_reset_peripheral_control(reset_id, 0);
}

void acts_reset_peripheral(int reset_id)
{
	acts_reset_peripheral_assert(reset_id);
	acts_reset_peripheral_deassert(reset_id);
}
