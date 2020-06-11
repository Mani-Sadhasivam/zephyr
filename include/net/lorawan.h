/*
 * Copyright (c) 2019 Manivannan Sadhasivam <mani@kernel.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_NET_LORAWAN_H_
#define ZEPHYR_INCLUDE_NET_LORAWAN_H_

/**
 * @file
 * @brief Public LoRaWAN APIs
 */

#include <zephyr/types.h>
#include <device.h>

/**
 * @brief LoRaWAN class types.
 */
enum lorawan_class {
	LORAWAN_CLASS_A = 0x00,
	LORAWAN_CLASS_B = 0x01,
	LORAWAN_CLASS_C = 0x02,
};

/**
 * @brief LoRaWAN activation types.
 */
enum lorawan_act_type {
	LORAWAN_ACT_OTAA = 0,
	LORAWAN_ACT_ABP,
};

/**
 * @brief LoRaWAN datarate types.
 */
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

struct lorawan_mib_config {
	enum lorawan_class lw_class;
	u8_t *dev_eui;
	u8_t *join_eui;
	u8_t *app_eui;
	u8_t *app_key;
	u8_t *nwk_key;
	u32_t system_max_rs_error;
	bool pub_nw;
	bool adr_enable;
};

/**
 * @brief Configure the LoRaWAN stack
 *
 * Configure the LoRaWAN stack using MIB (Mac Information Base) parameters.
 *
 * @param mib_config MIB configuration
 *
 * @return 0 if successful, negative errno code if failure
 */
int lorawan_config(struct lorawan_mib_config *mib_config);

/**
 * @brief Join the LoRaWAN network
 *
 * Join the LoRaWAN network using either OTAA or AWB.
 *
 * @param datarate   Datarate to be used for network join
 * @param mode       Activation mode
 *
 * @return 0 if successful, negative errno code if failure
 */
int lorawan_join_network(enum lorawan_datarate datarate,
			 enum lorawan_act_type mode);

/**
 * @brief Send data to the LoRaWAN network
 *
 * Send data to the connected LoRaWAN network.
 *
 * @param port       Port to be used for sending data. Must be set if the
 *                   payload is not empty.
 * @param datarate   Datarate to be used for sending data
 * @param data       Data buffer to be sent
 * @param len        Length of the buffer to be sent. Maximum length of this
 *                   buffer is 255 bytes but the actual payload size varies with
 *                   region and datarate.
 * @param confirm    Use confirmed messages
 * @param tries      Number of tries needed for sending the data in case the Ack
 *                   from server is not received
 *
 * @return 0 if successful, negative errno code if failure
 */
int lorawan_send(u8_t port, enum lorawan_datarate datarate,
		 u8_t *data, u8_t len, bool confirm, u8_t tries);
/**
 * @brief Check if there is uncollected received data from the LoRaWAN network
 *
 * Check if there is uncolllected received data from the LoRaWAN network
 *
 * @return 0 or positive number indicating the number of received messages
 * waiting to be read from the buffer, or negative errno code if failure
 */
int lorawan_receive_available();

/**
 * @brief Check if any uncollected received incoming data was discarded
 *
 * Check if any uncollected received incoming data was discarded due to
 * insufficient free space in the RX buffer.
 *
 * @return 0 or positive number indicating the number of discarded messages
 */
int lorawan_receive_discarded();

/**
 * @brief Read uncollected received data from the LoRaWAN network
 *
 * Read uncollected received data from the LoRaWAN network. This function will
 * return each received message only once - each read opration removes the
 * oldest downlink measage from the buffer of received unread messages.
 *
 * @param port       Port at which the message arrived
 * @param data       Data buffer where received data shall be placed
 * @param len        Size of the \p data buffer available. Messages with
 * 		     payloads larger than the buffer will be truncated
 *
 * @return 0 if successful, negative errno code if failure
 */
int lorawan_receive_read(u8_t *port, u8_t *data, u8_t len);

#endif	/* ZEPHYR_INCLUDE_NET_LORAWAN_H_ */
