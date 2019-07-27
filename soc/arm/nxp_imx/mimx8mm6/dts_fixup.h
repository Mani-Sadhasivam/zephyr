/*
 * Copyright (c) 2019, Manivannan Sadhasivam <mani@kernel.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* SoC level DTS fixup file */

#define DT_NUM_IRQ_PRIO_BITS	    	DT_ARM_V7M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

#define DT_MCUX_CCM_BASE_ADDRESS	DT_NXP_IMX_CCM_400FC000_BASE_ADDRESS
#define DT_MCUX_CCM_NAME		DT_NXP_IMX_CCM_400FC000_LABEL

#define DT_UART_MCUX_4_NAME		DT_NXP_KINETIS_IMX_UART_30A60000_LABEL
#define DT_UART_MCUX_4_BASE_ADDRESS	DT_NXP_KINETIS_IMX_UART_30A60000_BASE_ADDRESS
#define DT_UART_MCUX_4_BAUD_RATE	DT_NXP_KINETIS_IMX_UART_30A60000_CURRENT_SPEED
#define DT_UART_MCUX_4_IRQ_NUM		DT_NXP_KINETIS_IMX_UART_30A60000_IRQ_0
#define DT_UART_MCUX_4_IRQ_PRI		DT_NXP_KINETIS_IMX_UART_30A60000_IRQ_0_PRIORITY

#define DT_UART_MCUX_4_CLOCK_NAME	DT_NXP_KINETIS_IMX_UART_30A60000_CLOCK_CONTROLLER
#define DT_UART_MCUX_4_CLOCK_SUBSYS	DT_NXP_KINETIS_IMX_UART_30A60000_CLOCK_NAME

/* End of SoC Level DTS fixup file */
