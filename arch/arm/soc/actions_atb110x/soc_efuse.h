/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file efuse configuration macros for Actions SoC
 */

#ifndef	_ACTIONS_SOC_EFUSE_H_
#define	_ACTIONS_SOC_EFUSE_H_

#define EFUSE_REG_BASE      PMU_REG_BASE
#define EFUSE25_CTL         (EFUSE_REG_BASE + 0x60)
#define EFUSE_RD_CTL        (EFUSE_REG_BASE + 0x64)
#define EFUSE_WR_CTL        (EFUSE_REG_BASE + 0x68)
#define EFUSE_MANUAL_CTL    (EFUSE_REG_BASE + 0x6C)
#define EFUSE_PASSWORD      (EFUSE_REG_BASE + 0x70)
#define EFUSE_ADDRESS       (EFUSE_REG_BASE + 0x74)
#define EFUSE_DATA          (EFUSE_REG_BASE + 0x78)
#define PMU_DEBUG           (EFUSE_REG_BASE + 0x7C)

//--------------BITS LOCATION------------------------------------------//
#define EFUSE25_CTL_EFUSE25_DEBUG_EN      7
#define EFUSE25_CTL_EFUSE25_VOL_E         6
#define EFUSE25_CTL_EFUSE25_VOL_SHIFT     4
#define EFUSE25_CTL_EFUSE25_VOL_MASK      (0x7<<4)
#define EFUSE25_CTL_EFUSE25_PD_E          3
#define EFUSE25_CTL_EFUSE25_PD_SHIFT      2
#define EFUSE25_CTL_EFUSE25_PD_MASK       (0x3<<2)
#define EFUSE25_CTL_EN_EFUSE25_PD         1
#define EFUSE25_CTL_EN_EFUSE25            0

#define EFUSE_RD_CTL_DATA_STATE           4
#define EFUSE_RD_CTL_READ_CLKD_E          3
#define EFUSE_RD_CTL_READ_CLKD_SHIFT      2
#define EFUSE_RD_CTL_READ_CLKD_MASK       (0x3<<2)
#define EFUSE_RD_CTL_RD_PSD_OK            1
#define EFUSE_RD_CTL_AUTO_RD_EN           0

#define EFUSE_MANUAL_CTL_LOAD             3
#define EFUSE_MANUAL_CTL_STROBE           2
#define EFUSE_MANUAL_CTL_PGENB            1
#define EFUSE_MANUAL_CTL_CSB              0

#define EFUSE_PASSWORD_PASSWORD_E         31
#define EFUSE_PASSWORD_PASSWORD_SHIFT     0
#define EFUSE_PASSWORD_PASSWORD_MASK      (0xFFFFFFFF<<0)

#define EFUSE_ADDRESS_EFUSE_ADDR_E        9
#define EFUSE_ADDRESS_EFUSE_ADDR_SHIFT    0
#define EFUSE_ADDRESS_EFUSE_ADDR_MASK     (0x3FF<<0)

#define EFUSE_DATA_EFUSE_DATA_E           31
#define EFUSE_DATA_EFUSE_DATA_SHIFT       0
#define EFUSE_DATA_EFUSE_DATA_MASK        (0xFFFFFFFF<<0)

#define RPASSWORD	0x004b4559

#ifndef _ASMLANGUAGE

uint32_t acts_efuse_read(uint8_t offset);
uint32_t acts_efuse_read_val(uint8_t offset);
void acts_set_vd12_before_efuse_read(void);
void acts_set_vd12_after_efuse_read(void);

#endif /* _ASMLANGUAGE */

#endif /* _ACTIONS_SOC_EFUSE_H_	*/
