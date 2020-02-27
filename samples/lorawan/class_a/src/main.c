/*
 * Class A LoRaWAN sample application
 *
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <errno.h>
#include <lorawan.h>
#include <sys/util.h>
#include <zephyr.h>

#define LORAWAN_DEV_EUI			{ 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6,\
					  0x77, 0x88 }
#define LORAWAN_JOIN_EUI		{ 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02,\
					  0x4E, 0x50 }
#define LORAWAN_APP_EUI			{ 0x70, 0xB3, 0xD5, 0x7E, 0xD0, 0x02,\
					  0x4E, 0x50 }
#define LORAWAN_APP_KEY         	{ 0x8A, 0x74, 0xB4, 0x96, 0xF6, 0x6D,\
					  0xA6, 0xD2, 0xE7, 0x7E, 0x12, 0x3E,\
					  0xB2, 0x25, 0x0C, 0x5E }
#define LORAWAN_DEFAULT_DATARATE	LORAWAN_DR_0

#define MAX_DATA_LEN 10

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lorawan_class_a);

char data[MAX_DATA_LEN] = {'h', 'e', 'l', 'l', 'o', 'w', 'o', 'r', 'l', 'd'};

void main(void)
{
	struct device *lora_dev;
	struct lorawan_mib_config mib_config;
	int ret;
	u8_t dev_eui[] = LORAWAN_DEV_EUI;
	u8_t join_eui[] = LORAWAN_JOIN_EUI;
	u8_t app_eui[] = LORAWAN_APP_EUI;
	u8_t app_key[] = LORAWAN_APP_KEY;

	lora_dev = device_get_binding(DT_INST_0_SEMTECH_SX1276_LABEL);
	if (!lora_dev) {
		LOG_ERR("%s Device not found", DT_INST_0_SEMTECH_SX1276_LABEL);
		return;
	}

	mib_config.lw_class = LORAWAN_CLASS_A;
	mib_config.dev_eui = dev_eui;
	mib_config.join_eui = join_eui;
	mib_config.app_eui = app_eui;
	mib_config.app_key = app_key;
	mib_config.nwk_key = app_key;
	mib_config.adr_enable = true;
	mib_config.join_acc_delay1 = 5000;
	mib_config.join_acc_delay2 = 6000;
	mib_config.system_max_rs_error = 20;

	LOG_INF("Configuring MIB");
	ret = lorawan_config(&mib_config);
	if (ret < 0)
		return;

	LOG_INF("Joining network over OTAA");
	ret = lorawan_join_network(LORAWAN_DEFAULT_DATARATE, LORAWAN_ACT_OTAA);
	if (ret < 0)
		return;
	
	LOG_INF("Sending data...");
	ret = lorawan_send(2, LORAWAN_DEFAULT_DATARATE, data, MAX_DATA_LEN,
			   true, 1);
	if (ret < 0)
		return;

	LOG_INF("Data sent!");
}
