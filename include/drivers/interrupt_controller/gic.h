/*
 * Copyright 2019 Broadcom.
 * The term “Broadcom” refers to Broadcom Inc. and/or its subsidiaries
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief generic interrupt controller specific header
 *
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_GIC_H_
#define ZEPHYR_INCLUDE_DRIVERS_GIC_H_

#include <device.h>
#include <sys/slist.h>

#ifndef _ASMLANGUAGE

#ifdef __cplusplus
extern "C" {
#endif

#define	GIC_NS_DEFAULT_PRI	0xa0

#define GIC_SGI(x)		x
#define GIC_PPI(x)		(x + 16)
#define GIC_SPI(x)		(x + 32)

#define NP_ARCH_TIMER_INTID	GIC_PPI(14)

#ifdef __cplusplus
}
#endif

#endif /* _ASMLANGUAGE */

#endif /* ZEPHYR_INCLUDE_DRIVERS_GIC_H_ */
