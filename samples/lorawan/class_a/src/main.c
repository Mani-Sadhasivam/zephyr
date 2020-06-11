/*
 * Class A LoRaWAN sample application
 *
 * Copyright (c) 2019 Manivannan Sadhasivam <mani@kernel.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <net/lorawan.h>
#include <zephyr.h>

#define DEFAULT_RADIO_NODE DT_ALIAS(lora0)
BUILD_ASSERT(DT_HAS_NODE_STATUS_OKAY(DEFAULT_RADIO_NODE),
	     "No default LoRa radio specified in DT");
#define DEFAULT_RADIO DT_LABEL(DEFAULT_RADIO_NODE)

/* Customize based on network configuration */
#define LORAWAN_DEV_EUI			{ 0xDD, 0xEE, 0xAA, 0xDD, 0xBB, 0xEE,\
					  0xEE, 0xFF }
#define LORAWAN_JOIN_EUI		{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00 }
#define LORAWAN_APP_EUI			{ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,\
					  0x00, 0x00 }
#define LORAWAN_APP_KEY			{ 0x2B, 0x7E, 0x15, 0x16, 0x28, 0xAE,\
					  0xD2, 0xA6, 0xAB, 0xF7, 0x15, 0x88,\
					  0x09, 0xCF, 0x4F, 0x3C }
#define LORAWAN_DEFAULT_DATARATE	LORAWAN_DR_0

#define DELAY K_MSEC(5000)
#define DL_MSG_LEN 256

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lorawan_class_a);

char data[] = {'h', 'e', 'l', 'l', 'o', 'w', 'o', 'r', 'l', 'd'};

void main(void)
{
	struct device *lora_dev;
	struct lorawan_mib_config mib_config;
	int ret;
	int dl_msgs_no;
	int i;
	uint8_t port;
	uint8_t dl_msg[DL_MSG_LEN] = {'\0'};
	u8_t dev_eui[] = LORAWAN_DEV_EUI;
	u8_t join_eui[] = LORAWAN_JOIN_EUI;
	u8_t app_eui[] = LORAWAN_APP_EUI;
	u8_t app_key[] = LORAWAN_APP_KEY;

	lora_dev = device_get_binding(DEFAULT_RADIO);
	if (!lora_dev) {
		LOG_ERR("%s Device not found", DEFAULT_RADIO);
		return;
	}

	mib_config.lw_class = LORAWAN_CLASS_A;
	mib_config.dev_eui = dev_eui;
	mib_config.join_eui = join_eui;
	mib_config.app_eui = app_eui;
	mib_config.app_key = app_key;
	mib_config.nwk_key = app_key;
	mib_config.pub_nw = true; /* Connecting to a public network */
	mib_config.adr_enable = true;
	mib_config.system_max_rs_error = 20;

	LOG_INF("Configuring LoRaWAN stack");
	ret = lorawan_config(&mib_config);
	if (ret < 0) {
		LOG_ERR("lorawan_config failed: %d", ret);
		return;
	}

	ret = -1;
	
	while (ret < 0) {
	LOG_INF("Joining network over OTAA");
	ret = lorawan_join_network(LORAWAN_DEFAULT_DATARATE, LORAWAN_ACT_OTAA);
	if (ret < 0) {
		LOG_ERR("lorawan_join_network failed: %d", ret);
		k_sleep(K_MSEC(3000));
	} else {
		break;
	}
	}

	while (1) {
		k_sleep(DELAY);
		LOG_INF("Sending data...");
		ret = lorawan_send(2, LORAWAN_DEFAULT_DATARATE, data,
				   sizeof(data), true, 1);
		/*
		 * Note: The stack may return -EAGAIN if the provided data
		 * length exceeds the maximum possible one for the region and
		 * datarate. But since we are just sending the same data here,
		 * we'll just continue.
		 */
		if (ret == -EAGAIN) {
			LOG_ERR("lorawan_send failed: %d. Retrying...", ret);
			continue;
		}

		if (ret < 0) {
			LOG_ERR("lorawan_send failed: %d. Retrying...", ret);
			continue;
		}

		LOG_INF("Data sent!");
		k_sleep(K_MSEC(1000));

		dl_msgs_no = lorawan_receive_available();
		if (dl_msgs_no >= 0) {
			LOG_INF("%d downlink msgs available", dl_msgs_no);
			for (i=0; i<dl_msgs_no; i++){
				ret = lorawan_receive_read(&port, dl_msg, DL_MSG_LEN-1);
				if ( ret >= 0 ) {
					LOG_INF("Msg at port %d of length %d: %s",
						port,
						ret,
						log_strdup(dl_msg));
				}
			}
		}
	}
}