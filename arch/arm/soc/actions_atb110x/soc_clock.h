/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file peripheral clock configuration macros for Actions SoC
 */

#ifndef	_ACTIONS_SOC_CLOCK_H_
#define	_ACTIONS_SOC_CLOCK_H_

#define	CLOCK_ID_DMA                    0
#define	CLOCK_ID_SPICACHE               1
#define	CLOCK_ID_SPI0                   2
#define	CLOCK_ID_SPI1                   3
#define	CLOCK_ID_SPI2                   4
#define	CLOCK_ID_UART0                  5
#define	CLOCK_ID_UART1                  6
#define	CLOCK_ID_UART2                  7
#define	CLOCK_ID_I2C0                   8
#define	CLOCK_ID_I2C1                   9
#define	CLOCK_ID_PWM0                   10
#define	CLOCK_ID_PWM1                   11
#define	CLOCK_ID_PWM2                   12
#define	CLOCK_ID_PWM3                   13
#define	CLOCK_ID_PWM4                   14
#define	CLOCK_ID_KEY                    15
#define	CLOCK_ID_ADC                    16
#define	CLOCK_ID_I2STX                  17
#define	CLOCK_ID_I2SRX                  18
#define	CLOCK_ID_LRADC                  19
#define	CLOCK_ID_TIMER0                 20
#define	CLOCK_ID_TIMER1                 21
#define	CLOCK_ID_TIMER2                 22
#define	CLOCK_ID_TIMER                  23
#define	CLOCK_ID_BLE_HCLK_EN            24
#define	CLOCK_ID_BLE_HCLK_GATING        25
#define	CLOCK_ID_BLE_LLCC_CLK_REQ_O     26
#define	CLOCK_ID_HCL4HZ                 27
#define	CLOCK_ID_HCL32K                 28
#define	CLOCK_ID_EFUSE                  29
#define	CLOCK_ID_IRC_RXCLK              30
#define	CLOCK_ID_IRC_CLK                31

#define	CLOCK_ID_MAX_ID                 31

#define ACT_32M_CTL                     (CMU_DIGITAL_REG_BASE + 0x0004)
#define ACT_3M_CTL                      (CMU_DIGITAL_REG_BASE + 0x0008)
#define CMU_SYSCLK                      (CMU_DIGITAL_REG_BASE + 0x000C)

#define CMU_DEVRST                      (CMU_DIGITAL_REG_BASE + 0x0010)
#define CMU_DEVCLKEN                    (CMU_DIGITAL_REG_BASE + 0x0014)

#define CMU_TIMER0CLK                   (CMU_DIGITAL_REG_BASE + 0x003C)
#define CMU_TIMER1CLK                   (CMU_DIGITAL_REG_BASE + 0x0040)

#define ACT_32M_CTL_XTAL32M_EN          0

#define ACT_3M_CTL_RC3M_OK              5
#define ACT_3M_CTL_RSEL_E               4
#define ACT_3M_CTL_RSEL_SHIFT           1
#define ACT_3M_CTL_RSEL_MASK            (0xF<<1)
#define ACT_3M_CTL_RC3MEN               0

#define CMU_SYSCLK_BLE_HCLK_DIV_E       17
#define CMU_SYSCLK_BLE_HCLK_DIV_SHIFT   16
#define CMU_SYSCLK_BLE_HCLK_DIV_MASK    (0x3<<16)
#define CMU_SYSCLK_BLE_HCLK_SEL         15
#define CMU_SYSCLK_HOSC_A               13
#define CMU_SYSCLK_LOSC_A               12
#define CMU_SYSCLK_APB_DIV_E            11
#define CMU_SYSCLK_APB_DIV_SHIFT        10
#define CMU_SYSCLK_APB_DIV_MASK         (0x3<<10)
#define CMU_SYSCLK_AHB_DIV              9
#define CMU_SYSCLK_CPU_COREPLL          8
#define CMU_SYSCLK_CPU_32M              7
#define CMU_SYSCLK_CPU_32K              6
#define CMU_SYSCLK_CPU_3M               5
#define CMU_SYSCLK_SLEEP_HFCLK_SEL      4
#define CMU_SYSCLK_PCLKG                2
#define CMU_SYSCLK_CPU_CLK_SEL_E        1
#define CMU_SYSCLK_CPU_CLK_SEL_SHIFT    0
#define CMU_SYSCLK_CPU_CLK_SEL_MASK     (0x3<<0)

#ifndef _ASMLANGUAGE

void acts_clock_peripheral_enable(int clock_id);
void acts_clock_peripheral_disable(int clock_id);

void acts_request_rc_3M(bool ena);
void acts_request_pclk_gating(bool ena);

#endif /* _ASMLANGUAGE */

#endif /* _ACTIONS_SOC_CLOCK_H_	*/
