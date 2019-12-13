/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <device.h>
#include <drivers/uart.h>
#include <soc.h>

#define UTS                     0xb4
#define UTXD                    0x40
#define UCR1                    0x80
#define UCR1_UARTEN             (1<<0)
#define UTS_TXEMPTY             (1 << 6)

struct uart_dummy_config {
	u32_t base;
};

static int uart_dummy_poll_in(struct device *dev, unsigned char *c)
{
	return 0;
}

static void uart_dummy_poll_out(struct device *dev, unsigned char c)
{
	const struct uart_dummy_config *config = dev->config->config_info;

	while(!(sys_read32(config->base + UTS) & UTS_TXEMPTY));

	if ((sys_read32(config->base + UCR1) & UCR1_UARTEN) != UCR1_UARTEN)
		return;

	sys_write32(c, config->base + UTXD);
}

static int uart_dummy_init(struct device *dev)
{
	return 0;
}

static const struct uart_driver_api uart_dummy_driver_api = {
	.poll_in = uart_dummy_poll_in,
	.poll_out = uart_dummy_poll_out,
};

static const struct uart_dummy_config uart_dummy_0_config = {
	.base = DT_UART_DUMMY_30890000_BASE_ADDRESS,
};

DEVICE_AND_API_INIT(uart_0, DT_UART_DUMMY_30890000_LABEL,
		    &uart_dummy_init,
		    NULL, &uart_dummy_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_dummy_driver_api);
