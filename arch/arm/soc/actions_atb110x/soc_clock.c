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

static void acts_clock_peripheral_control(int clock_id, int enable)
{
	unsigned int key;

	if (clock_id > CLOCK_ID_MAX_ID)
		return;

	key = irq_lock();

	if (enable)
		sys_write32(sys_read32(CMU_DEVCLKEN_REG) | (1 << clock_id),
			    CMU_DEVCLKEN_REG);
	else
		sys_write32(sys_read32(CMU_DEVCLKEN_REG) & ~(1 << clock_id),
			    CMU_DEVCLKEN_REG);

	irq_unlock(key);
}

void acts_clock_peripheral_enable(int clock_id)
{
	acts_clock_peripheral_control(clock_id, 1);
}

void acts_clock_peripheral_disable(int clock_id)
{
	acts_clock_peripheral_control(clock_id, 0);
}

static uint8_t rc_3M_cnt = 1;
void acts_request_rc_3M(bool ena)
{
	unsigned int key;

	key = irq_lock();
	if (ena) {
		if (rc_3M_cnt == 0) {
			acts_request_vd12(true);
			sys_write32(sys_read32(ACT_3M_CTL) | (1 << ACT_3M_CTL_RC3MEN), ACT_3M_CTL);
			while((sys_read32(ACT_3M_CTL) & (1 << ACT_3M_CTL_RC3M_OK)) == 0);
		}
		rc_3M_cnt++;
	} else {
		rc_3M_cnt--;
		if (rc_3M_cnt == 0) {
			sys_write32(sys_read32(ACT_3M_CTL) & ~(1 << ACT_3M_CTL_RC3MEN), ACT_3M_CTL);
			acts_request_vd12(false);
		}
	}
	irq_unlock(key);
}
