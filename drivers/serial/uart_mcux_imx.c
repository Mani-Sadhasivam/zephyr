/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <device.h>
#include <drivers/uart.h>
#include <drivers/clock_control.h>
#include <fsl_uart.h>
#include <soc.h>

struct uart_mcux_config {
	UART_Type *base;
	char *clock_name;
	clock_control_subsys_t clock_subsys;
	u32_t baud_rate;
};

struct uart_mcux_data {
};

static int uart_mcux_poll_in(struct device *dev, unsigned char *c)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	int ret = -1;

	if (UART_GetStatusFlag(config->base, kUART_RxDataReadyFlag) || UART_GetStatusFlag(config->base, kUART_RxOverrunFlag)) {
		*c = UART_ReadByte(config->base);
		ret = 0;
	}

	return ret;
}

static void uart_mcux_poll_out(struct device *dev, unsigned char c)
{
	const struct uart_mcux_config *config = dev->config->config_info;

	while (!(UART_GetStatusFlag(config->base, kUART_TxReadyFlag))) {
	}

	UART_WriteByte(config->base, c);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_mcux_fifo_fill(struct device *dev, const u8_t *tx_data,
			       int len)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u8_t num_tx = 0U;

	while ((len - num_tx > 0) &&
	       (UART_GetStatusFlag(config->base) & kUART_TxDataRegEmptyFlag)) {

		UART_WriteByte(config->base, tx_data[num_tx++]);
	}

	return num_tx;
}

static int uart_mcux_fifo_read(struct device *dev, u8_t *rx_data,
			       const int len)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u8_t num_rx = 0U;

	while ((len - num_rx > 0) &&
	       (UART_GetStatusFlag(config->base) & kUART_RxDataRegFullFlag)) {

		rx_data[num_rx++] = UART_ReadByte(config->base);
	}

	return num_rx;
}

static void uart_mcux_irq_tx_enable(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u32_t mask = kUART_TxDataRegEmptyInterruptEnable;

	UART_EnableInterrupts(config->base, mask);
}

static void uart_mcux_irq_tx_disable(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u32_t mask = kUART_TxDataRegEmptyInterruptEnable;

	UART_DisableInterrupts(config->base, mask);
}

static int uart_mcux_irq_tx_complete(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u32_t flags = UART_GetStatusFlag(config->base);

	return (flags & kUART_TxDataRegEmptyFlag) != 0U;
}

static int uart_mcux_irq_tx_ready(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u32_t mask = kUART_TxDataRegEmptyInterruptEnable;

	return (UART_GetEnabledInterrupts(config->base) & mask)
		&& uart_mcux_irq_tx_complete(dev);
}

static void uart_mcux_irq_rx_enable(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u32_t mask = kUART_RxDataRegFullInterruptEnable;

	UART_EnableInterrupts(config->base, mask);
}

static void uart_mcux_irq_rx_disable(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u32_t mask = kUART_RxDataRegFullInterruptEnable;

	UART_DisableInterrupts(config->base, mask);
}

static int uart_mcux_irq_rx_full(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u32_t flags = UART_GetStatusFlag(config->base);

	return (flags & kUART_RxDataRegFullFlag) != 0U;
}

static int uart_mcux_irq_rx_ready(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u32_t mask = kUART_RxDataRegFullInterruptEnable;

	return (UART_GetEnabledInterrupts(config->base) & mask)
		&& uart_mcux_irq_rx_full(dev);
}

static void uart_mcux_irq_err_enable(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u32_t mask = kUART_NoiseErrorInterruptEnable |
			kUART_FramingErrorInterruptEnable |
			kUART_ParityErrorInterruptEnable;

	UART_EnableInterrupts(config->base, mask);
}

static void uart_mcux_irq_err_disable(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	u32_t mask = kUART_NoiseErrorInterruptEnable |
			kUART_FramingErrorInterruptEnable |
			kUART_ParityErrorInterruptEnable;

	UART_DisableInterrupts(config->base, mask);
}

static int uart_mcux_irq_is_pending(struct device *dev)
{
	return uart_mcux_irq_tx_ready(dev) || uart_mcux_irq_rx_ready(dev);
}

static int uart_mcux_irq_update(struct device *dev)
{
	return 1;
}

static void uart_mcux_irq_callback_set(struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_mcux_data *data = dev->driver_data;

	data->callback = cb;
	data->cb_data = cb_data;
}

static void uart_mcux_isr(void *arg)
{
	struct device *dev = arg;
	struct uart_mcux_data *data = dev->driver_data;

	if (data->callback) {
		data->callback(data->cb_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int uart_mcux_init(struct device *dev)
{
	const struct uart_mcux_config *config = dev->config->config_info;
	uart_config_t uart_config;
	struct device *clock_dev;
	u32_t clock_freq;

	clock_dev = device_get_binding(config->clock_name);
	if (clock_dev == NULL) {
		return -EINVAL;
	}

	if (clock_control_get_rate(clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

	UART_GetDefaultConfig(&uart_config);
	uart_config.enableTx = true;
	uart_config.enableRx = true;
	uart_config.baudRate_Bps = config->baud_rate;

	UART_Init(config->base, &uart_config, clock_freq);

	return 0;
}

static const struct uart_driver_api uart_mcux_driver_api = {
	.poll_in = uart_mcux_poll_in,
	.poll_out = uart_mcux_poll_out,
};

#ifdef CONFIG_UART_MCUX_IMX_4

static const struct uart_mcux_config uart_mcux_4_config = {
	.base = UART4,
	.clock_name = DT_UART_MCUX_4_CLOCK_NAME,
	.clock_subsys = (clock_control_subsys_t)DT_UART_MCUX_4_CLOCK_SUBSYS,
	.baud_rate = DT_UART_MCUX_4_BAUD_RATE,
};

static struct uart_mcux_data uart_mcux_4_data;

DEVICE_AND_API_INIT(uart_4, DT_UART_MCUX_4_NAME,
		    &uart_mcux_init,
		    &uart_mcux_4_data, &uart_mcux_4_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &uart_mcux_driver_api);

#endif /* CONFIG_UART_MCUX_IMX_4 */
