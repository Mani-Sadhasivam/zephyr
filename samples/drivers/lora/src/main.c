/*
 * Copyright (c) 2018 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <errno.h>
#include <lora.h>
#include <misc/util.h>
#include <zephyr.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lora_log);

char data[26] = {25,25,25,25,5,5,5,5,5,5,5,5,5,25,5,5,5,5,5,5,5,25,5,5,5,5};

void main(void)
{
	struct device *lora_dev;
	struct lora_modem_config config;
	int ret;

	lora_dev = device_get_binding(DT_INST_0_SEMTECH_SX1276_LABEL);
	if (!lora_dev) {
		LOG_ERR("%s Device not found", DT_INST_0_SEMTECH_SX1276_LABEL);
		return;
	}

	config.frequency = 868100000;
	config.bandwidth = BW_125_KHZ;
	config.spreading_factor = SF_10;
	config.preamble_len = 8;
	config.coding_rate = CR_4_5;
	config.tx_power = 4;

	ret = lora_config(lora_dev, &config);
	if (ret < 0) {
		LOG_ERR("LoRa config failed");
		return;
	}

//	while (1) {
		LOG_INF("sending");
		ret = lora_send(lora_dev, data, 26);
		if (ret < 0) {
			LOG_ERR("LoRa send failed");
			return;
		}

		k_sleep(2000);
//	}

	while(1);
}
