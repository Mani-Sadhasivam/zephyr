/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/gpio.h>
#include <spi.h>
#include <misc/util.h>
#include <net/lora.h>
#include <zephyr.h>
#include <sx1276/sx1276.h>

#define LOG_LEVEL CONFIG_LORA_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(sx1276);

#define SX1276_MAX_DIO		5

#define GPIO_RESET_PIN		DT_INST_0_SEMTECH_SX1276_RESET_GPIOS_PIN
#define GPIO_CS_PIN		DT_INST_0_SEMTECH_SX1276_CS_GPIOS_PIN
#define CLOCK_FREQ		DT_INST_0_SEMTECH_SX1276_CLOCK_FREQUENCY

#define SX1276_REG_VERSION			0x42

int sx1276_dio_pins[SX1276_MAX_DIO] = {
	DT_INST_0_SEMTECH_SX1276_DIO_GPIOS_PIN_0,
	DT_INST_0_SEMTECH_SX1276_DIO_GPIOS_PIN_1,
	DT_INST_0_SEMTECH_SX1276_DIO_GPIOS_PIN_2,
	DT_INST_0_SEMTECH_SX1276_DIO_GPIOS_PIN_3,
	DT_INST_0_SEMTECH_SX1276_DIO_GPIOS_PIN_4,
};

static char sx1276_dio_ports[SX1276_MAX_DIO][6] = {
	DT_INST_0_SEMTECH_SX1276_DIO_GPIOS_CONTROLLER_0,
	DT_INST_0_SEMTECH_SX1276_DIO_GPIOS_CONTROLLER_1,
	DT_INST_0_SEMTECH_SX1276_DIO_GPIOS_CONTROLLER_2,
	DT_INST_0_SEMTECH_SX1276_DIO_GPIOS_CONTROLLER_3,
	DT_INST_0_SEMTECH_SX1276_DIO_GPIOS_CONTROLLER_4,
};

struct sx1276_data {
	struct device *spi;
	struct spi_config spi_cfg;
	struct device *dio_dev[SX1276_MAX_DIO];
	struct gpio_callback irq_cb;
	struct k_sem data_sem;
} dev_data;

bool SX1276CheckRfFrequency( uint32_t frequency )
{
    // Implement check. Currently all frequencies are supported
    return true;
}

static void sx1276_irq_callback(struct device *dev,
				struct gpio_callback *cb, u32_t pins)
{
}

static int sx1276_transceive(u8_t reg, bool write, void *data, size_t length)
{
	const struct spi_buf buf[2] = {
		{
			.buf = &reg,
			.len = 1
		},
		{
			.buf = data,
			.len = length
		}
	};
	struct spi_buf_set tx = {
		.buffers = buf,
		.count = 2,
	};

	if (!write) {
		const struct spi_buf_set rx = {
			.buffers = buf,
			.count = 2
		};

		return spi_transceive(dev_data.spi, &dev_data.spi_cfg,
				&tx, &rx);
	}

	return spi_write(dev_data.spi, &dev_data.spi_cfg, &tx);
}

int sx1276_read(u8_t reg_addr, u8_t *data, u8_t len)
{
	return sx1276_transceive(reg_addr, false, data, len);
}

int sx1276_write(u8_t reg_addr, u8_t byte)
{
	return sx1276_transceive(reg_addr | BIT(7), true, &byte, 1);
}

void SX1276WriteBuffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
	int ret, i;

	for (i = 0; i < size; i++) {
		ret = sx1276_write(addr, buffer[i]);
		if (ret < 0) {
			LOG_ERR("Unable to read address: %x", addr);
			return;
		}
	}
}

void SX1276ReadBuffer(uint16_t addr, uint8_t *buffer, uint8_t size)
{
	int ret, i;
	u8_t regval;

	for (i = 0; i < size; i++) {
		ret = sx1276_read(addr, &regval, 1);
		if (ret < 0) {
			LOG_ERR("Unable to read address: %x", addr);
			return;
		}

		buffer[i] = regval;
	}
}

static int sx1276_lora_send(struct device *dev, u8_t *data, u32_t data_len)
{
	return 0;
}

static int sx1276_lora_recv(struct device *dev, u8_t *data)
{
	return 0;
}

static int sx1276_lora_config(struct device *dev,
			      struct lora_modem_config *config)
{
	return 0;
}

static int sx1276_lora_init(struct device *dev)
{
	static struct spi_cs_control spi_cs;
	struct device *reset_dev;
	int ret;
	unsigned int i;
	u8_t regval;

	dev_data.spi = device_get_binding(DT_INST_0_SEMTECH_SX1276_BUS_NAME);
	if (!dev_data.spi) {
		LOG_ERR("spi device not found: %s",
			    DT_INST_0_SEMTECH_SX1276_BUS_NAME);
		return -EINVAL;
	}

	dev_data.spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	dev_data.spi_cfg.frequency = DT_INST_0_SEMTECH_SX1276_SPI_MAX_FREQUENCY;
	dev_data.spi_cfg.slave = DT_INST_0_SEMTECH_SX1276_BASE_ADDRESS;

	spi_cs.gpio_pin = GPIO_CS_PIN,
	spi_cs.gpio_dev = device_get_binding(
			DT_INST_0_SEMTECH_SX1276_CS_GPIOS_CONTROLLER);
	if (!spi_cs.gpio_dev) {
		LOG_ERR("Failed to initialize CS GPIO driver: %s",
		       DT_INST_0_SEMTECH_SX1276_CS_GPIOS_CONTROLLER);
		return -EIO;
	}

	dev_data.spi_cfg.cs = &spi_cs;

	/* Setup DIO gpios */
	for (i = 0; i < SX1276_MAX_DIO; i++) {
		dev_data.dio_dev[i] = device_get_binding(sx1276_dio_ports[i]);
		if (dev_data.dio_dev[i] == NULL) {
			LOG_ERR("Cannot get pointer to %s device",
				sx1276_dio_ports[i]);
			return -EINVAL;
		}

		gpio_pin_configure(dev_data.dio_dev[i], sx1276_dio_pins[i],
				   GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
				   GPIO_INT_DEBOUNCE | GPIO_INT_ACTIVE_HIGH);

		gpio_init_callback(&dev_data.irq_cb,
				   sx1276_irq_callback,
				   BIT(sx1276_dio_pins[i]));

		if (gpio_add_callback(dev_data.dio_dev[i], &dev_data.irq_cb) < 0) {
			LOG_ERR("Could not set gpio callback.");
			return -EIO;
		}
		gpio_pin_enable_callback(dev_data.dio_dev[i], sx1276_dio_pins[i]);
	}

	/* Setup Reset gpio */
	reset_dev = device_get_binding(
			DT_INST_0_SEMTECH_SX1276_RESET_GPIOS_CONTROLLER);
	if (!reset_dev) {
		LOG_ERR("Failed to initialize Reset GPIO driver: %s",
		       DT_INST_0_SEMTECH_SX1276_RESET_GPIOS_CONTROLLER);
		return -EIO;
	}

	ret = gpio_pin_configure(reset_dev, GPIO_RESET_PIN, GPIO_DIR_OUT);

	/* Perform soft reset */
	gpio_pin_write(reset_dev, GPIO_RESET_PIN, 0);
	k_sleep(100);
	gpio_pin_write(reset_dev, GPIO_RESET_PIN, 1);
	k_sleep(100);

	ret = sx1276_read(SX1276_REG_VERSION, &regval, 1);
	if (ret < 0) {
		LOG_ERR("Unable to read version info");
		return -EIO;
	}

	k_sem_init(&dev_data.data_sem, 0, UINT_MAX);

	LOG_INF("SX1276 Version:%02x found", regval);

	return 0;
}

/* Initialize Radio driver callbacks */
const struct Radio_s Radio = {
	.Init = SX1276Init,
	.GetStatus = SX1276GetStatus,
	.SetModem = SX1276SetModem,
	.SetChannel = SX1276SetChannel,
	.IsChannelFree = SX1276IsChannelFree,
	.Random = SX1276Random,
	.SetRxConfig = SX1276SetRxConfig,
	.SetTxConfig = SX1276SetTxConfig,
	.CheckRfFrequency = SX1276CheckRfFrequency,
	.TimeOnAir = SX1276GetTimeOnAir,
	.Send = SX1276Send,
	.Sleep = SX1276SetSleep,
	.Standby = SX1276SetStby,
	.Rx = SX1276SetRx,
	.StartCad = SX1276StartCad,
	.SetTxContinuousWave = SX1276SetTxContinuousWave,
	.Rssi = SX1276ReadRssi,
	.Write = SX1276Write,
	.Read = SX1276Read,
	.WriteBuffer = SX1276WriteBuffer,
	.ReadBuffer = SX1276ReadBuffer,
	.SetMaxPayloadLength = SX1276SetMaxPayloadLength,
	.SetPublicNetwork = SX1276SetPublicNetwork,
	.GetWakeupTime = SX1276GetWakeupTime,
	.IrqProcess = NULL,
	.RxBoosted = NULL,
	.SetRxDutyCycle = NULL,
};

static const struct lora_driver_api sx1276_lora_api = {
	.config = sx1276_lora_config,
	.send = sx1276_lora_send,
	.recv = sx1276_lora_recv,
};

DEVICE_AND_API_INIT(sx1276_lora, DT_INST_0_SEMTECH_SX1276_LABEL,
		    &sx1276_lora_init, NULL,
		    NULL, POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,
		    &sx1276_lora_api);
