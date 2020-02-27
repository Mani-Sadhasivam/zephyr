/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_LORAWAN_H_
#define ZEPHYR_INCLUDE_LORAWAN_H_

/**
 * @file
 * @brief Public LoRaWAN APIs
 */

#include <zephyr/types.h>
#include <device.h>

enum lorawan_class {
	LORAWAN_CLASS_A = 0x00,
	LORAWAN_CLASS_B = 0x01,
	LORAWAN_CLASS_C = 0x02,
};

enum lorawan_act_type {
	LORAWAN_ACT_OTAA = 0,
	LORAWAN_ACT_ABP,
};

enum lorawan_datarate {
	LORAWAN_DR_0 = 0,
	LORAWAN_DR_1,
	LORAWAN_DR_2,
	LORAWAN_DR_3,
	LORAWAN_DR_4,
	LORAWAN_DR_5,
	LORAWAN_DR_6,
	LORAWAN_DR_7,
	LORAWAN_DR_8,
	LORAWAN_DR_9,
	LORAWAN_DR_10,
	LORAWAN_DR_11,
	LORAWAN_DR_12,
	LORAWAN_DR_13,
	LORAWAN_DR_14,
	LORAWAN_DR_15,
};

/* TODO: Remove if not req */
enum lorawan_mac_status {
	LORAWAN_MAC_STATUS_OK,
	LORAWAN_MAC_STATUS_BUSY,
	LORAWAN_MAC_STATUS_SERVICE_UNKNOWN,
	LORAWAN_MAC_STATUS_PARAMETER_INVALID,
	LORAWAN_MAC_STATUS_FREQUENCY_INVALID,
	LORAWAN_MAC_STATUS_DATARATE_INVALID,
	LORAWAN_MAC_STATUS_FREQ_AND_DR_INVALID,
	LORAWAN_MAC_STATUS_NO_NETWORK_JOINED,
	LORAWAN_MAC_STATUS_LENGTH_ERROR,
	LORAWAN_MAC_STATUS_DEVICE_OFF,
	LORAWAN_MAC_STATUS_REGION_NOT_SUPPORTED,
};

struct lorawan_mib_config {
	enum lorawan_class lw_class;
	u8_t *dev_eui;
	u8_t *join_eui;
	u8_t *app_eui;
	u8_t *app_key;
	u8_t *nwk_key;
	u32_t join_acc_delay1;
	u32_t join_acc_delay2;
	bool adr_enable;
	u32_t system_max_rs_error;
};

/* TODO: Convert to APIs */
extern int lorawan_config(struct lorawan_mib_config *mib_config);
extern int lorawan_join_network(enum lorawan_datarate datarate,
				enum lorawan_act_type mode);
extern int lorawan_send(u8_t port, enum lorawan_datarate datarate,
			u8_t *data, u8_t len, bool confirm, u8_t tries);

#endif	/* ZEPHYR_INCLUDE_LORAWAN_H_ */
