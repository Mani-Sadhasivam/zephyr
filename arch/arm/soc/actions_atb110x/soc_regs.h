/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file register address for Actions SoC
 */

#ifndef	_ACTIONS_SOC_REGS_H_
#define	_ACTIONS_SOC_REGS_H_

#define CMU_DIGITAL_REG_BASE  0x40002000

#define CMU_DEVRST_REG        0x40002010
#define CMU_DEVCLKEN_REG      0x40002014
#define CMU_SPI0CLK_REG       0x40002018
#define CMU_SPI1CLK_REG       0x4000201c
#define CMU_SPI2CLK_REG       0x40002020
#define CMU_PWM0CLK_REG       0x40002024
#define CMU_PWM1CLK_REG       0x40002028
#define CMU_PWM2CLK_REG       0x4000202c
#define CMU_PWM3CLK_REG       0x40002030
#define CMU_PWM4CLK_REG       0x40002034
#define CMU_AUDIOCLK_REG      0x40002038

#define CMU_ANALOG_REG_BASE   0x40002100

#define RTC_REG_BASE          0x40004000
#define	TIMER_REG_BASE        0x40004100
#define	HCL_REG_BASE          0x40004200

#define PMU_REG_BASE          0x40008000
#define ADC_REG_BASE          0x40008010

#define MEMCTL_REG_BASE       0x40009000
#define DMA_REG_BASE          0x4000a000

#define I2C0_REG_BASE         0x4000b000
#define I2C1_REG_BASE         0x4000c000

#define UART0_REG_BASE        0x4000d000
#define UART1_REG_BASE        0x4000e000
#define UART2_REG_BASE        0x4000f000

#define SPI0_REG_BASE         0x40011000
#define SPI1_REG_BASE         0x40012000
#define SPI2_REG_BASE         0x40013000

#define GPIO_REG_BASE         0x40016000
#define PWM_REG_BASE          0x40017000
#define KEY_REG_BASE          0x40018000

#endif /* _ACTIONS_SOC_REGS_H_	*/
