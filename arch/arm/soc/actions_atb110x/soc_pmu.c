/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file pmu(vdd/vd12) interface for Actions SoC
 */

#include <kernel.h>
#include "soc.h"

void acts_request_vd12_pd(bool ena)
{
	uint32_t key;

	key = irq_lock();
	if (ena)
		sys_write32(sys_read32(VD12_CTL) | (1 << VD12_CTL_VD12PD_EN),
				VD12_CTL);
	else
		sys_write32(sys_read32(VD12_CTL) & ~(1 << VD12_CTL_VD12PD_EN),
				VD12_CTL);

	irq_unlock(key);
}

void acts_request_vd12_largebias(bool ena)
{
	uint32_t key;

	key = irq_lock();
	if (ena)
		sys_write32(sys_read32(VD12_CTL) | (1 << VD12_CTL_VD12_LGBIAS_EN),
				VD12_CTL);
	else
		sys_write32(sys_read32(VD12_CTL) & ~(1 << VD12_CTL_VD12_LGBIAS_EN),
				VD12_CTL);

	irq_unlock(key);
}

void acts_request_vd12(bool ena)
{
  /* VD12 always on */
}

void acts_set_vdd(vdd_val_t val)
{
	uint32_t key;

	key = irq_lock();

	sys_write32((sys_read32(VDD_CTL) & ~VDD_CTL_VDD_SET_MASK) | val, VDD_CTL);

	irq_unlock(key);
}
