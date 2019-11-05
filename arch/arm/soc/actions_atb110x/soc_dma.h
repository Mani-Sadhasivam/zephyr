/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file dma configuration macros for Actions SoC
 */

#ifndef	_ACTIONS_SOC_DMA_H_
#define	_ACTIONS_SOC_DMA_H_

#define	DMA_ID_MEMORY     0
#define	DMA_ID_SPI0       1
#define	DMA_ID_SPI1       2
#define	DMA_ID_SPI2       3
#define	DMA_ID_UART0      4
#define	DMA_ID_UART1      5
#define	DMA_ID_UART2      6
#define	DMA_ID_I2C0       7
#define	DMA_ID_I2C1       8
#define	DMA_ID_I2S        9
#define	DMA_ID_PWM        10

#define	DMA_ID_MAX_ID     10

#define DMA_GLOB_OFFS     0x0000
#define DMA_CHAN_OFFS     0x0100

#define DMA_CHAN(base, id)    ((base) + DMA_CHAN_OFFS + (id) * 0x100)
#define DMA_GLOBAL(base)      ((base) + DMA_GLOB_OFFS)

#define DMA_ACTS_MAX_CHANNELS 4	/* Number of streams per controller */

/* Maximum data sent in single transfer (Bytes) */
#define DMA_ACTS_MAX_DATA_ITEMS  0x1fff

#ifndef _ASMLANGUAGE

#endif /* _ASMLANGUAGE */

#endif /* _ACTIONS_SOC_DMA_H_	*/
