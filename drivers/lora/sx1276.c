/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <gpio.h>
#include <spi.h>
#include <lora.h>
#include <misc/util.h>
#include <zephyr.h>

#define LOG_LEVEL CONFIG_LORA_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(sx1276);

#define SX1276_REG_FIFO				0x00
#define SX1276_REG_OPMODE			0x01
#define SX1276_REG_FRF_MSB			0x06
#define SX1276_REG_FRF_MID			0x07
#define SX1276_REG_FRF_LSB			0x08
#define SX1276_REG_PA_CONFIG			0x09
#define SX1276_REG_FIFO_ADDR_PTR		0x0d
#define SX1276_REG_FIFO_TX_BASE_ADDR		0x0e
#define SX1276_REG_IRQ_FLAGS_MASK		0x11
#define SX1276_REG_IRQ_FLAGS			0x12
#define SX1276_REG_MODEM_CONFIG1		0x1d
#define SX1276_REG_MODEM_CONFIG2		0x1e
#define SX1276_REG_PAYLOAD_LENGTH		0x22
#define SX1276_REG_SYNC_WORD			0x39
#define SX1276_REG_DIO_MAPPING1			0x40
#define SX1276_REG_DIO_MAPPING2			0x41
#define SX1276_REG_VERSION			0x42
#define SX1276_REG_PA_DAC			0x4d

#define SX1276_OPMODE_LONG_RANGE_MODE		BIT(7)
#define SX1276_OPMODE_LOW_FREQUENCY_MODE_ON	BIT(3)
#define SX1276_OPMODE_MODE_MASK			GENMASK(2, 0)
#define SX1276_OPMODE_MODE_SLEEP		(0x0 << 0)
#define SX1276_OPMODE_MODE_STDBY		(0x1 << 0)
#define SX1276_OPMODE_MODE_TX			(0x3 << 0)
#define SX1276_OPMODE_MODE_RXCONTINUOUS		(0x5 << 0)
#define SX1276_OPMODE_MODE_RXSINGLE		(0x6 << 0)

#define SX1276_MODEM_CONFIG1_BW_MASK		GENMASK(7, 4)
#define SX1276_MODEM_CONFIG2_SF_MASK		GENMASK(7, 4)
#define SX1276_PA_CONFIG_PA_BOOST		BIT(7)

#define LORA_REG_IRQ_FLAGS_TX_DONE		BIT(3)

#define REG_DIO_MAPPING1_DIO0_MASK	GENMASK(7, 6)

#define GPIO_RESET_PIN			DT_SEMTECH_SX1276_0_RESET_GPIOS_PIN
#define GPIO_CS_PIN			DT_SEMTECH_SX1276_0_CS_GPIO_PIN
#define CLOCK_FREQ			DT_SEMTECH_SX1276_0_CLOCK_FREQUENCY

#define SX1276_PA_RFO				0
#define SX1276_PA_BOOST				1

struct sx1276_data {
	struct device *spi;
	struct spi_config spi_cfg;
};

struct _sx1276_power {
	u8_t min;
	u8_t max;
};

static struct _sx1276_power sx1276_power[2] = {
	[SX1276_PA_RFO] = {
		.min = 0,
		.max = 14,
	},
	[SX1276_PA_BOOST] = {
		.min = 2,
		.max = 20,
	},
};
static int sx1276_transceive(struct device *dev, u8_t reg,
			     bool write, void *data, size_t length)
{
	struct sx1276_data *dev_data = dev->driver_data;
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

		return spi_transceive(dev_data->spi, &dev_data->spi_cfg, &tx, &rx);
	}

	return spi_write(dev_data->spi, &dev_data->spi_cfg, &tx);
}

int sx1276_read(struct device *dev, u8_t reg_addr, u8_t *data, u8_t len)
{
	return sx1276_transceive(dev, reg_addr, false, data, len);
}

int sx1276_write(struct device *dev, u8_t reg_addr, u8_t byte)
{
	return sx1276_transceive(dev, reg_addr | BIT(7), true, &byte, 1);
}

int sx1276_fifo_write(struct device *dev, u8_t *data, u32_t data_len)
{
	int ret, i;

	for (i = 0; i < data_len; i++) {
		ret = sx1276_write(dev, SX1276_REG_FIFO, data[i]);
		if (ret < 0) {
			LOG_ERR("Unable to write FIFO");
			return -EIO;
		}
	}

	return 0;
}

static int sx1276_lora_send(struct device *dev, u8_t *data, u32_t data_len)
{
	int ret;
	u8_t regval, fifo_ptr;

	/* Set standby mode */
	ret = sx1276_read(dev, SX1276_REG_OPMODE, &regval, 1);
	if (ret < 0) {
		LOG_ERR("Unable to read OPMODE");
		return -EIO;
	}

	regval &= ~SX1276_OPMODE_MODE_MASK;
	regval |= SX1276_OPMODE_MODE_STDBY;
	ret = sx1276_write(dev, SX1276_REG_OPMODE, regval); 
	if (ret < 0) {
		LOG_ERR("Unable to write OPMODE");
		return -EIO;
	}

	ret = sx1276_read(dev, SX1276_REG_FIFO_TX_BASE_ADDR, &fifo_ptr, 1);
	if (ret < 0) {
		LOG_ERR("Unable to read FIFO Tx base addr");
		return -EIO;
	}

	ret = sx1276_write(dev, SX1276_REG_FIFO_ADDR_PTR, fifo_ptr);
	if (ret < 0) {
		LOG_ERR("Unable to write FIFO Tx addr pointer");
		return -EIO;
	}

	ret = sx1276_write(dev, SX1276_REG_PAYLOAD_LENGTH, data_len);
	if (ret < 0) {
		LOG_ERR("Unable to write payload length");
		return -EIO;
	}

	ret = sx1276_fifo_write(dev, data, data_len);
	if (ret < 0)
		return ret;

	ret = sx1276_read(dev, SX1276_REG_OPMODE, &regval, 1);
	if (ret < 0) {
		LOG_ERR("Unable to read OPMODE");
		return -EIO;
	}

	regval &= ~SX1276_OPMODE_MODE_MASK;
	regval |= SX1276_OPMODE_MODE_TX;
	ret = sx1276_write(dev, SX1276_REG_OPMODE, regval); 
	if (ret < 0) {
		LOG_ERR("Unable to write OPMODE");
		return -EIO;
	}

	return 0;
}

static int sx1276_lora_config(struct device *dev,
			      struct lora_modem_config *config)
{
	unsigned long long freq_rf;
	int ret;
	u8_t regval, pa_config;

	/* Set LoRa and enter sleep mode */
	regval = SX1276_OPMODE_LONG_RANGE_MODE | SX1276_OPMODE_MODE_SLEEP;
	ret = sx1276_write(dev, SX1276_REG_OPMODE, regval); 
	if (ret < 0) {
		LOG_ERR("Unable to write OPMODE");
		return -EIO;
	}

	/* Set frequency */		
	freq_rf = config->frequency;;
	freq_rf *= (1 << 19);
	freq_rf /= CLOCK_FREQ;
	LOG_INF("FRF: %x", (unsigned int) freq_rf);

	ret = sx1276_write(dev, SX1276_REG_FRF_MSB, (freq_rf >> 16 & 0xFF));
	if (!ret)
		ret = sx1276_write(dev, SX1276_REG_FRF_MID, (freq_rf >> 8 & 0xFF));
	if (!ret)
		ret = sx1276_write(dev, SX1276_REG_FRF_LSB, (freq_rf & 0xFF));
	if (ret) {
		LOG_ERR("Unable to write RF carrier frequency");
		return -EIO;
	}

	switch (config->bandwidth) {
	case BW_125_KHZ:
	case BW_250_KHZ:
	case BW_500_KHZ:
		break;
	default:
		LOG_ERR("Bandwidth not supported");
		return -EINVAL;
	}

	/* Set bandwidth */
	ret = sx1276_read(dev, SX1276_REG_MODEM_CONFIG1, &regval, 1);
	if (ret < 0) {
		LOG_ERR("Unable to read Modem Config1");
		return -EIO;
	}

	regval &= ~SX1276_MODEM_CONFIG1_BW_MASK;
	regval |= (config->bandwidth << 4);
	ret = sx1276_write(dev, SX1276_REG_MODEM_CONFIG1, regval); 
	if (ret < 0) {
		LOG_ERR("Unable to write Modem Config1");
		return -EIO;
	}

	/* Set spreading factor */
	ret = sx1276_read(dev, SX1276_REG_MODEM_CONFIG2, &regval, 1);
	if (ret < 0) {
		LOG_ERR("Unable to read Modem Config2");
		return -EIO;
	}

	regval &= ~SX1276_MODEM_CONFIG2_SF_MASK;
	regval |= (config->spreading_factor << 4);
	ret = sx1276_write(dev, SX1276_REG_MODEM_CONFIG2, regval); 
	if (ret < 0) {
		LOG_ERR("Unable to write Modem Config2");
		return -EIO;
	}

	/* Set output power */
	ret = sx1276_read(dev, SX1276_REG_PA_CONFIG, &pa_config, 1);
	if (ret < 0) {
		LOG_ERR("Unable to read version PA config");
		return -EIO;
	}

	pa_config &= ~GENMASK(3, 0);
#if defined CONFIG_PA_RFO_PIN
	pa_config &= ~SX1276_PA_CONFIG_PA_BOOST;
	if (config->tx_power < sx1276_power[SX1276_PA_RFO].min)
		config->tx_power = sx1276_power[SX1276_PA_RFO].min;
	if (config->tx_power > sx1276_power[SX1276_PA_RFO].max)
		config->tx_power = sx1276_power[SX1276_PA_RFO].max;
#else
	pa_config |= SX1276_PA_CONFIG_PA_BOOST;
	if (config->tx_power > sx1276_power[SX1276_PA_BOOST].max -3) {
		if (config->tx_power > sx1276_power[SX1276_PA_BOOST].max)
			config->tx_power = sx1276_power[SX1276_PA_BOOST].max;

		config->tx_power -= 3;
		ret = sx1276_write(dev, SX1276_REG_PA_DAC, 0x87);
		if (ret < 0) {
			LOG_ERR("Unable to write PADAC");
			return -EIO;
		}
	} else {
		if (config->tx_power < sx1276_power[SX1276_PA_BOOST].min)
			config->tx_power = sx1276_power[SX1276_PA_BOOST].min;

		ret = sx1276_write(dev, SX1276_REG_PA_DAC, 0x84);
		if (ret < 0) {
			LOG_ERR("Unable to write PADAC");
			return -EIO;
		}
	}
#endif
	pa_config |= (config->tx_power & 0x0F);
	ret = sx1276_write(dev, SX1276_REG_PA_CONFIG, pa_config); 
	if (ret < 0) {
		LOG_ERR("Unable to write PA config");
		return -EIO;
	}

	return 0;
}

static int sx1276_lora_init(struct device *dev)
{
	struct sx1276_data *data = dev->driver_data;
	static struct spi_cs_control spi_cs;
	struct device *reset_dev;
	int ret;
	u8_t regval;

	data->spi = device_get_binding(DT_SEMTECH_SX1276_0_BUS_NAME);
	if (!data->spi) {
		LOG_ERR("spi device not found: %s",
			    DT_SEMTECH_SX1276_0_BUS_NAME);
		return -EINVAL;
	}

	data->spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	data->spi_cfg.frequency = DT_SEMTECH_SX1276_0_SPI_MAX_FREQUENCY;
	data->spi_cfg.slave = DT_SEMTECH_SX1276_0_BASE_ADDRESS;

	spi_cs.gpio_pin = GPIO_CS_PIN,
	spi_cs.gpio_dev = device_get_binding(
			DT_SEMTECH_SX1276_0_CS_GPIO_CONTROLLER);
	if (!spi_cs.gpio_dev) {
		LOG_ERR("Failed to initialize CS GPIO driver: %s",
		       DT_SEMTECH_SX1276_0_CS_GPIO_CONTROLLER);
		return -EIO;
	}

	data->spi_cfg.cs = &spi_cs;

	reset_dev = device_get_binding(
			DT_SEMTECH_SX1276_0_RESET_GPIOS_CONTROLLER);
	if (!reset_dev) {
		LOG_ERR("Failed to initialize Reset GPIO driver: %s",
		       DT_SEMTECH_SX1276_0_RESET_GPIOS_CONTROLLER);
		return -EIO;
	}

	ret = gpio_pin_configure(reset_dev, GPIO_RESET_PIN, GPIO_DIR_OUT);

	/* Perform soft reset */
	gpio_pin_write(reset_dev, GPIO_RESET_PIN, 0);
	k_sleep(100);
	gpio_pin_write(reset_dev, GPIO_RESET_PIN, 1);
	k_sleep(100);

	ret = sx1276_read(dev, SX1276_REG_VERSION, &regval, 1);
	if (ret < 0) {
		LOG_ERR("Unable to read version info");
		return -EIO;
	}

	LOG_INF("SX1276 Version:%02x found", regval);

	return 0;
}

static struct sx1276_data sx1276_lora_data;

static const struct lora_driver_api sx1276_lora_api = {
	.config = sx1276_lora_config,
	.send = sx1276_lora_send,
};

DEVICE_AND_API_INIT(sx1276_lora, DT_SEMTECH_SX1276_0_LABEL,
		    &sx1276_lora_init, &sx1276_lora_data,
		    NULL, POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,
		    &sx1276_lora_api);
