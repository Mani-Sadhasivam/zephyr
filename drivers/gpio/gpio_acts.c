/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief GPIO driver for Actions SoC
 */

#include <errno.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <irq.h>
#include <gpio.h>
#include <soc.h>
#include "gpio_utils.h"

/**
 * @brief driver data
 */
struct acts_gpio_data {
	/* Enabled INT pins generating a cb */
	u32_t cb_pins;
	/* user ISR cb */
	sys_slist_t cb;
};

typedef void (*gpio_config_irq_t)(struct device *dev);
struct acts_gpio_config {
	u32_t base;
	u32_t irq_num;
	gpio_config_irq_t config_func;
};

/**
 * @brief Configurate pin or port
 *
 * @param dev Device struct
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param flags Flags of pin or port
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_acts_config(struct device *dev, int access_op,
			    u32_t pin, int flags)
{
	const struct acts_gpio_config *info = dev->config->config_info;
	u32_t key, val;

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= GPIO_MAX_PIN_NUM)
		return -EINVAL;

	val = GPIO_CTL_MFP_GPIO | GPIO_CTL_INTC_MASK;

	/* config pull up/down */
	if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_UP) {
		val |= GPIO_CTL_PULLUP;
	} else if ((flags & GPIO_PUD_MASK) == GPIO_PUD_PULL_DOWN) {
		val |= GPIO_CTL_PULLDOWN;
	}

	/* config in/out */
	if ((flags & GPIO_DIR_MASK) == GPIO_DIR_OUT) {
		val |= GPIO_CTL_GPIO_OUTEN;
	} else {
		val |= GPIO_CTL_GPIO_INEN;
	}

	if (flags & GPIO_INT) {
		if (flags & GPIO_INT_EDGE) {
			if (flags & GPIO_INT_DOUBLE_EDGE)
				val |= GPIO_CTL_INC_TRIGGER_DUAL_EDGE;
			else if (flags & GPIO_INT_ACTIVE_HIGH)
				val |= GPIO_CTL_INC_TRIGGER_RISING_EDGE;
			else
				val |= GPIO_CTL_INC_TRIGGER_FALLING_EDGE;
		} else {
			if (flags & GPIO_INT_ACTIVE_HIGH)
				val |= GPIO_CTL_INC_TRIGGER_HIGH_LEVEL;
			else
				val |= GPIO_CTL_INC_TRIGGER_LOW_LEVEL;
		}
	}

	key = irq_lock();

	sys_write32(val, GPIO_REG_CTL(info->base, pin));

	if ((flags & GPIO_DIR_MASK) == GPIO_DIR_OUT) {
		/* initial gpio value, normal == 0, invert == 1 */
		if ((flags & GPIO_POL_MASK) == GPIO_POL_INV)
			sys_write32(GPIO_BIT(pin), GPIO_REG_BSR(info->base, pin));
		else
			sys_write32(GPIO_BIT(pin), GPIO_REG_BRR(info->base, pin));
	}

	irq_unlock(key);

	return 0;
}

/**
 * @brief Set the pin or port output
 *
 * @param dev Device struct
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value to set (0 or 1)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_acts_write(struct device *dev, int access_op,
			   u32_t pin, u32_t value)
{
	const struct acts_gpio_config *info = dev->config->config_info;
	u32_t key;

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= GPIO_MAX_PIN_NUM)
		return -EINVAL;

	key = irq_lock();

	if (value) {
		/* output high level */
		sys_write32(GPIO_BIT(pin), GPIO_REG_BSR(info->base, pin));
	} else {
		/* output low level */
		sys_write32(GPIO_BIT(pin), GPIO_REG_BRR(info->base, pin));
	}

	irq_unlock(key);

	return 0;
}

/**
 * @brief Read the pin or port status
 *
 * @param dev Device struct
 * @param access_op Access operation (pin or port)
 * @param pin The pin number
 * @param value Value of input pin(s)
 *
 * @return 0 if successful, failed otherwise
 */
static int gpio_acts_read(struct device *dev, int access_op,
			  u32_t pin, u32_t *value)
{
	const struct acts_gpio_config *info = dev->config->config_info;

	if (access_op != GPIO_ACCESS_BY_PIN)
		return -ENOTSUP;

	if (pin >= GPIO_MAX_PIN_NUM)
		return -EINVAL;

	*value = !!(sys_read32(GPIO_REG_IDAT(info->base, pin)) & GPIO_BIT(pin));

	return 0;
}

/**
 * @brief GPIO interrupt callback
 */
void gpio_acts_isr(void *arg)
{
	struct device *dev = arg;
	const struct acts_gpio_config *info = dev->config->config_info;
	struct acts_gpio_data *data = dev->driver_data;
	u32_t reg_pending, pending;

	reg_pending = GPIO_REG_IRQ_PD(info->base, 0);
	pending = sys_read32(reg_pending);

	_gpio_fire_callbacks(&data->cb, dev, pending);

	/* clear IRQ pending */
	sys_write32(pending, reg_pending);

	/* FIXME: wait util pending is cleared */
	while ((sys_read32(reg_pending) & pending) == pending)
		;
}

static int gpio_acts_manage_callback(struct device *dev,
				     struct gpio_callback *callback,
				     bool set)
{
	struct acts_gpio_data *data = dev->driver_data;

	_gpio_manage_callback(&data->cb, callback, set);

	return 0;
}

static int gpio_acts_enable_callback(struct device *dev,
				     int access_op, u32_t pin)
{
	const struct acts_gpio_config *info = dev->config->config_info;
	struct acts_gpio_data *data = dev->driver_data;
	u32_t key, ctl_reg;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	key = irq_lock();

	/* clear old irq pending */
	sys_write32(GPIO_BIT(pin), GPIO_REG_IRQ_PD(info->base, pin));

	/* enable gpio irq */
	ctl_reg = GPIO_REG_CTL(info->base, pin);
	sys_write32(sys_read32(ctl_reg) | GPIO_CTL_INTC_EN, ctl_reg);

	data->cb_pins |= BIT(pin);

	irq_unlock(key);

	return 0;
}

static int gpio_acts_disable_callback(struct device *dev,
				      int access_op, u32_t pin)
{
	const struct acts_gpio_config *info = dev->config->config_info;
	struct acts_gpio_data *data = dev->driver_data;
	u32_t key, ctl_reg;

	if (access_op != GPIO_ACCESS_BY_PIN) {
		return -ENOTSUP;
	}

	key = irq_lock();

	data->cb_pins &= ~BIT(pin);

	/* disable gpio irq */
	ctl_reg = GPIO_REG_CTL(info->base, pin);
	sys_write32(sys_read32(ctl_reg) & ~GPIO_CTL_INTC_EN, ctl_reg);

	irq_unlock(key);

	return 0;
}

const struct gpio_driver_api gpio_acts_drv_api = {
	.config = gpio_acts_config,
	.write = gpio_acts_write,
	.read = gpio_acts_read,
	.manage_callback = gpio_acts_manage_callback,
	.enable_callback = gpio_acts_enable_callback,
	.disable_callback = gpio_acts_disable_callback,
};

/**
 * @brief Initialization function of GPIO
 *
 * @param dev Device struct
 * @return 0 if successful, failed otherwise.
 */
int gpio_acts_init(struct device *dev)
{
	const struct acts_gpio_config *info = dev->config->config_info;

	info->config_func(dev);

	return 0;
}

static void gpio_acts_config_irq(struct device *dev);

static const struct acts_gpio_config acts_gpio_config_info0 = {
	.base = GPIO_REG_BASE,
	.irq_num = IRQ_ID_GPIO,
	.config_func = gpio_acts_config_irq,
};

static struct acts_gpio_data acts_gpio_data0;

DEVICE_AND_API_INIT(gpio_acts, CONFIG_GPIO_ACTS_DRV_NAME,
		    gpio_acts_init, &acts_gpio_data0, &acts_gpio_config_info0,
		    PRE_KERNEL_1, CONFIG_GPIO_ACTS_INIT_PRIORITY,
		    &gpio_acts_drv_api);

static void gpio_acts_config_irq(struct device *dev)
{
	const struct acts_gpio_config *info = dev->config->config_info;

	IRQ_CONNECT(IRQ_ID_GPIO, 1, gpio_acts_isr,
		    DEVICE_GET(gpio_acts), 0);
	irq_enable(info->irq_num);
}
