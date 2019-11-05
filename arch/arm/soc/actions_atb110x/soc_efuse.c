/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file efuse interface for Actions SoC
 */

#include <kernel.h>
#include "soc.h"

uint32_t acts_efuse_read_val(uint8_t offset)
{
	uint32_t val;
	uint32_t key;

	key = irq_lock();

	/* disable EFUSE25 */
	sys_write32(sys_read32(EFUSE25_CTL) & ~(0x1 << EFUSE25_CTL_EN_EFUSE25), EFUSE25_CTL);

	/* config PGENB for read */
	sys_write32(sys_read32(EFUSE_MANUAL_CTL) | (0x1 << EFUSE_MANUAL_CTL_PGENB), EFUSE_MANUAL_CTL);

	/* write read_address */
	sys_write32(offset, EFUSE_ADDRESS);

	/* ENABLE READ */
	sys_write32(sys_read32(EFUSE_RD_CTL) | (0x1 << EFUSE_RD_CTL_AUTO_RD_EN), EFUSE_RD_CTL);

	/* write readpassword */
	sys_write32(RPASSWORD, EFUSE_DATA);

	/* wait data ready */
	while((sys_read32(EFUSE_RD_CTL) & (0x1 << EFUSE_RD_CTL_DATA_STATE)) == 0);

	/* read data */
	val = sys_read32(EFUSE_DATA);

	/* clear data_ok */
	sys_write32(sys_read32(EFUSE_RD_CTL) | (0x1 << EFUSE_RD_CTL_DATA_STATE), EFUSE_RD_CTL);

	irq_unlock(key);
	return val;
}

void acts_set_vd12_before_efuse_read(void)
{
	acts_request_vd12_largebias(true);
	acts_request_vd12(true);
	acts_request_vd12_pd(true);
}

void acts_set_vd12_after_efuse_read(void)
{
	acts_request_vd12_pd(false);
	acts_request_vd12_largebias(false);
	acts_request_vd12(false);
}

uint32_t acts_efuse_read(uint8_t offset)
{
	uint32_t val;
	uint32_t key;

	key = irq_lock();

	/* set vd12 before read efuse */
	acts_set_vd12_before_efuse_read();
 
	val = acts_efuse_read_val(offset);

	/* restore vd12 after efuse */
	acts_set_vd12_after_efuse_read();

	irq_unlock(key);
	return val;
}
