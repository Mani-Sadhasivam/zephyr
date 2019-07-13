/*
 * Copyright (c) 2019 Linaro Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <gpio.h>
#include <init.h>
#include <kernel.h>
#include <pinmux.h>
#include <sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

static const struct pin_config pinconf[] = {
#ifdef CONFIG_UART_1
	{STM32_PIN_PA9,  STM32L1X_PINMUX_FUNC_PA9_USART1_TX},
	{STM32_PIN_PA10, STM32L1X_PINMUX_FUNC_PA10_USART1_RX},
#endif	/* CONFIG_UART_1 */
#ifdef CONFIG_UART_2
	{STM32_PIN_PA2, STM32L1X_PINMUX_FUNC_PA2_USART2_TX},
	{STM32_PIN_PA15, STM32L1X_PINMUX_FUNC_PA15_USART2_RX},
#endif	/* CONFIG_UART_2 */
#ifdef CONFIG_I2C_1
	{STM32_PIN_PB8, STM32L1X_PINMUX_FUNC_PB8_I2C1_SCL},
	{STM32_PIN_PB9, STM32L1X_PINMUX_FUNC_PB9_I2C1_SDA},
#endif /* CONFIG_I2C_1 */
#ifdef CONFIG_SPI_1
	{STM32_PIN_PA5, STM32L1X_PINMUX_FUNC_PA5_SPI1_SCK},
	{STM32_PIN_PA6, STM32L1X_PINMUX_FUNC_PA6_SPI1_MISO},
	{STM32_PIN_PA7, STM32L1X_PINMUX_FUNC_PA7_SPI1_MOSI},
#endif /* CONFIG_SPI_1 */
	{STM32_PIN_PA4, STM32_PUSHPULL_PULLUP},
	{STM32_PIN_PA11, STM32_PUSHPULL_PULLUP},
	{STM32_PIN_PB6, STM32_PUSHPULL_PULLUP},
	{STM32_PIN_PB7, STM32_PUSHPULL_PULLUP},
//	{STM32_PIN_PH1, STM32_PUSHPULL_NOPULL},
};

static int pinmux_stm32_init(struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	struct device *gpioa =
	       device_get_binding(DT_ST_STM32_GPIO_40020000_LABEL);
	struct device *gpiob =
	       device_get_binding(DT_ST_STM32_GPIO_40020400_LABEL);
	struct device *gpioh =
	       device_get_binding(DT_ST_STM32_GPIO_40021400_LABEL);

	gpio_pin_configure(gpioa, 4, GPIO_DIR_OUT);
	gpio_pin_write(gpioa, 4, 1);

	gpio_pin_configure(gpiob, 6, GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 6, 0);

	gpio_pin_configure(gpiob, 7, GPIO_DIR_OUT);
	gpio_pin_write(gpiob, 6, 0);

	gpio_pin_configure(gpioh, 1, GPIO_DIR_OUT);
	gpio_pin_write(gpioh, 1, 1);

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1, CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
