/*
 * Copyright (c) 2019 Manivannan Sadhasivam <mani@kernel.org>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <errno.h>
#include <net/lorawan.h>
#include <zephyr.h>

#if CONFIG_LORAWAN_USE_RX_RING_BUFFER
#include <sys/ring_buffer.h>
#endif //def CONFIG_LORAWAN_USE_RX_RING_BUFFER

#include <LoRaMac.h>

#ifdef CONFIG_LORAMAC_REGION_AS923
	#define LORAWAN_REGION LORAMAC_REGION_AS923
#elif CONFIG_LORAMAC_REGION_AU915
	#define LORAWAN_REGION LORAMAC_REGION_AU915
#elif CONFIG_LORAMAC_REGION_CN470
	#define LORAWAN_REGION LORAMAC_REGION_CN470
#elif CONFIG_LORAMAC_REGION_CN779
	#define LORAWAN_REGION LORAMAC_REGION_CN779
#elif CONFIG_LORAMAC_REGION_EU433
	#define LORAWAN_REGION LORAMAC_REGION_EU433
#elif CONFIG_LORAMAC_REGION_EU868
	#define LORAWAN_REGION LORAMAC_REGION_EU868
#elif CONFIG_LORAMAC_REGION_KR920
	#define LORAWAN_REGION LORAMAC_REGION_KR920
#elif CONFIG_LORAMAC_REGION_IN865
	#define LORAWAN_REGION LORAMAC_REGION_IN865
#elif CONFIG_LORAMAC_REGION_US915
	#define LORAWAN_REGION LORAMAC_REGION_US915
#elif CONFIG_LORAMAC_REGION_RU864
	#define LORAWAN_REGION LORAMAC_REGION_RU864
#elif
	#error "Atleast one LoRaWAN region should be selected"
#endif

#define CEIL_32(x) ((unsigned int)(((((x))+31U)/32U)*32U))
#define SIZE_BYTES_TO_WORDS(x) (CEIL_32(x*8)/32)


#define LORAWAN_PLD_MAX_SIZE_BYTES	242U
#define LORAWAN_PLD_MAX_SIZE_WORDS	SIZE_BYTES_TO_WORDS(LORAWAN_PLD_MAX_SIZE_BYTES)

#define LOG_LEVEL CONFIG_LORAWAN_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lorawan);

K_SEM_DEFINE(mlme_confirm_sem, 0, 1);
K_SEM_DEFINE(mcps_confirm_sem, 0, 1);

K_MUTEX_DEFINE(lorawan_join_mutex);
K_MUTEX_DEFINE(lorawan_send_mutex);

#if CONFIG_LORAWAN_USE_RX_RING_BUFFER

#define RX_RING_BUF_SIZE_WORDS (CONFIG_LORAWAN_RX_RING_BUFFER_MAX_SIZE_WORDS)
struct rx_ring_buf {
    struct ring_buf rb;
    u32_t buffer[RX_RING_BUF_SIZE_WORDS];
};

struct rx_ring_buf rx_buf;

#else

uint8_t rx_buffer[LORAWAN_PLD_MAX_SIZE_BYTES];
uint8_t rx_buffer_stored_len;
uint8_t rx_buffer_port;

#endif //def CONFIG_LORAWAN_USE_RX_RING_BUFFER

static uint16_t rx_buf_avail_elem;
static uint16_t rx_buf_discarded_elem;
#if CONFIG_LORAWAN_USE_RX_RING_BUFFER
static uint32_t payload_tmp[LORAWAN_PLD_MAX_SIZE_WORDS];
static uint8_t payload_tmp_size = LORAWAN_PLD_MAX_SIZE_WORDS;
#endif
const char *status2str(int status)
{
	switch (status) {
	case LORAMAC_STATUS_OK:
		return "OK";
	case LORAMAC_STATUS_BUSY:
		return "Busy";
	case LORAMAC_STATUS_SERVICE_UNKNOWN:
		return "Service unknown";
	case LORAMAC_STATUS_PARAMETER_INVALID:
		return "Parameter invalid";
	case LORAMAC_STATUS_FREQUENCY_INVALID:
		return "Frequency invalid";
	case LORAMAC_STATUS_DATARATE_INVALID:
		return "Datarate invalid";
	case LORAMAC_STATUS_FREQ_AND_DR_INVALID:
		return "Frequency or datarate invalid";
	case LORAMAC_STATUS_NO_NETWORK_JOINED:
		return "No network joined";
	case LORAMAC_STATUS_LENGTH_ERROR:
		return "Length error";
	case LORAMAC_STATUS_REGION_NOT_SUPPORTED:
		return "Region not supported";
	case LORAMAC_STATUS_SKIPPED_APP_DATA:
		return "Skipped APP data";
	case LORAMAC_STATUS_DUTYCYCLE_RESTRICTED:
		return "Duty-cycle restricted";
	case LORAMAC_STATUS_NO_CHANNEL_FOUND:
		return "No channel found";
	case LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND:
		return "No free channel found";
	case LORAMAC_STATUS_BUSY_BEACON_RESERVED_TIME:
		return "Busy beacon reserved time";
	case LORAMAC_STATUS_BUSY_PING_SLOT_WINDOW_TIME:
		return "Busy ping-slot window time";
	case LORAMAC_STATUS_BUSY_UPLINK_COLLISION:
		return "Busy uplink collision";
	case LORAMAC_STATUS_CRYPTO_ERROR:
		return "Crypto error";
	case LORAMAC_STATUS_FCNT_HANDLER_ERROR:
		return "FCnt handler error";
	case LORAMAC_STATUS_MAC_COMMAD_ERROR:
		return "MAC command error";
	case LORAMAC_STATUS_CLASS_B_ERROR:
		return "ClassB error";
	case LORAMAC_STATUS_CONFIRM_QUEUE_ERROR:
		return "Confirm queue error";
	case LORAMAC_STATUS_MC_GROUP_UNDEFINED:
		return "Multicast group undefined";
	case LORAMAC_STATUS_ERROR:
		return "Unknown error";
	default:
		return NULL;
	}
}

const char *eventinfo2str(int status)
{
	switch (status) {
	case LORAMAC_EVENT_INFO_STATUS_OK:
		return "OK";
	case LORAMAC_EVENT_INFO_STATUS_ERROR:
		return "Error";
	case LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT:
		return "Tx timeout";
	case LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT:
		return "Rx 1 timeout";
	case LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT:
		return "Rx 2 timeout";
	case LORAMAC_EVENT_INFO_STATUS_RX1_ERROR:
		return "Rx1 error";
	case LORAMAC_EVENT_INFO_STATUS_RX2_ERROR:
		return "Rx2 error";
	case LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL:
		return "Join failed";
	case LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED:
		return "Downlink repeated";
	case LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR:
		return "Tx DR payload size error";
	case LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS:
		return "Downlink too many frames loss";
	case LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL:
		return "Address fail";
	case LORAMAC_EVENT_INFO_STATUS_MIC_FAIL:
		return "MIC fail";
	case LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL:
		return "Multicast fail";
	case LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED:
		return "Beacon locked";
	case LORAMAC_EVENT_INFO_STATUS_BEACON_LOST:
		return "Beacon lost";
	case LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND:
		return "Beacon not found";
	default:
		return NULL;
	}
}

/*
 * MAC status and Event status to Zephyr error code conversion.
 * Direct mapping is not possible as statuses often indicate the domain from
 * which the error originated rather than its cause or meaning. -EINVAL has been
 * used as a general error code because those usually result from incorrect
 * configuration.
 */
const int mac_status_to_errno[] = {
	0,			/* LORAMAC_STATUS_OK */
	-EBUSY,			/* LORAMAC_STATUS_BUSY */
	-ENOPROTOOPT,		/* LORAMAC_STATUS_SERVICE_UNKNOWN */
	-EINVAL,		/* LORAMAC_STATUS_PARAMETER_INVALID */
	-EINVAL,		/* LORAMAC_STATUS_FREQUENCY_INVALID */
	-EINVAL,		/* LORAMAC_STATUS_DATARATE_INVALID */
	-EINVAL,		/* LORAMAC_STATUS_FREQ_AND_DR_INVALID */
	-ENOTCONN,		/* LORAMAC_STATUS_NO_NETWORK_JOINED */
	-EMSGSIZE,		/* LORAMAC_STATUS_LENGTH_ERROR */
	-EPFNOSUPPORT,		/* LORAMAC_STATUS_REGION_NOT_SUPPORTED */
	-EMSGSIZE,		/* LORAMAC_STATUS_SKIPPED_APP_DATA */
	-ECONNREFUSED,		/* LORAMAC_STATUS_DUTYCYCLE_RESTRICTED */
	-ENOTCONN,		/* LORAMAC_STATUS_NO_CHANNEL_FOUND */
	-ENOTCONN,		/* LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND */
	-EBUSY,			/* LORAMAC_STATUS_BUSY_BEACON_RESERVED_TIME */
	-EBUSY,			/* LORAMAC_STATUS_BUSY_PING_SLOT_WINDOW_TIME */
	-EBUSY,			/* LORAMAC_STATUS_BUSY_UPLINK_COLLISION */
	-EINVAL,		/* LORAMAC_STATUS_CRYPTO_ERROR */
	-EINVAL,		/* LORAMAC_STATUS_FCNT_HANDLER_ERROR */
	-EINVAL,		/* LORAMAC_STATUS_MAC_COMMAD_ERROR */
	-EINVAL,		/* LORAMAC_STATUS_CLASS_B_ERROR */
	-EINVAL,		/* LORAMAC_STATUS_CONFIRM_QUEUE_ERROR */
	-EINVAL			/* LORAMAC_STATUS_MC_GROUP_UNDEFINED */
};

const int mac_event_info_to_errno[] = {
	0,			/* LORAMAC_EVENT_INFO_STATUS_OK */
	-EINVAL,		/* LORAMAC_EVENT_INFO_STATUS_ERROR */
	-ETIMEDOUT,		/* LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT */
	-ETIMEDOUT,		/* LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT */
	-ETIMEDOUT,		/* LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT */
	-EINVAL,		/* LORAMAC_EVENT_INFO_STATUS_RX1_ERROR */
	-EINVAL,		/* LORAMAC_EVENT_INFO_STATUS_RX2_ERROR */
	-EINVAL,		/* LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL */
	-ECONNRESET,		/* LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED */
	-EMSGSIZE,		/* LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR */
	-ECONNRESET,		/* LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS */
	-EACCES,		/* LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL */
	-EACCES,		/* LORAMAC_EVENT_INFO_STATUS_MIC_FAIL */
	-EINVAL,		/* LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL */
	-EINVAL,		/* LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED */
	-EINVAL,		/* LORAMAC_EVENT_INFO_STATUS_BEACON_LOST */
	-EINVAL			/* LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND */
};

static LoRaMacPrimitives_t macPrimitives;
static LoRaMacCallback_t macCallbacks;

static LoRaMacEventInfoStatus_t last_mcps_confirm_status;
static LoRaMacEventInfoStatus_t last_mlme_confirm_status;
static LoRaMacEventInfoStatus_t last_mcps_indication_status;
static LoRaMacEventInfoStatus_t last_mlme_indication_status;

/**
 * @brief Read a downlink message from the ring buffer.
 *
 * Read a downlink message from the ring buffer and place its payload in the
 * provided container. The message is consumed in the process. If the container
 * is shorter than the available payload any excess data will be discarded and
 * only as much of the payload as possible will be output.
 *
 * @param port Container for the message's port
 * @param payload Container for the message's payload
 * @param len Lenght in bytes of the \p payload container.
 *
 * @retval non-negative integer indicating the number of read payload bytes
 *         successfully copied to the provided container
 * @retval -EAGAIN No messages in the buffer.
 */
static int rx_buf_get(u8_t *port, u8_t *payload, u16_t len)
{
	int len_to_copy;

#if CONFIG_LORAWAN_USE_RX_RING_BUFFER
	int ret;
	u16_t rx_buffer_stored_len;

	payload_tmp_size = LORAWAN_PLD_MAX_SIZE_WORDS;
	ret = ring_buf_item_get(&rx_buf.rb, &rx_buffer_stored_len,
				port, payload_tmp, &payload_tmp_size);
	payload_tmp_size = LORAWAN_PLD_MAX_SIZE_WORDS;

	if ( ret < 0 ) {
		return ret;
	}

	rx_buf_avail_elem--;
	len_to_copy = rx_buffer_stored_len < len ? rx_buffer_stored_len : len;
	memcpy(payload, payload_tmp, len_to_copy);
#else
	len_to_copy = rx_buffer_stored_len < len ? rx_buffer_stored_len : len;
	rx_buf_avail_elem = 0;
	*port = rx_buffer_port; 
	rx_buffer_port = 0;
	memcpy(payload, rx_buffer, len_to_copy);
#endif
	return len_to_copy;
}

/**
 * @brief Put a downlink message into the ring buffer.
 *
 * Put a downlink message intothe ring buffer. If the buffer doesn not have
 * sufficient free space oldest messages will be discarded for as long as
 * possible until sufficient space is available.
 *
 * @param port Message's port number
 * @param payload Container for the message's payload
 * @param len \p payload size in bytes.
 *
 * @retval 0 On success
 * @retval -EMSGSIZE Message too large for the buffer
*/
static int rx_buf_put(u8_t port, u8_t *payload, u16_t len)
{
#if CONFIG_LORAWAN_USE_RX_RING_BUFFER
	int ret;
	u16_t type_tmp;
	u8_t val_tmp;
	memcpy(payload_tmp, payload, len);

	ret = ring_buf_item_put(&rx_buf.rb, len, port, payload_tmp,
				SIZE_BYTES_TO_WORDS(len));

	while (ret == -EMSGSIZE) {
		/* not enough room for the data item -> remove the oldes one */
		payload_tmp_size = LORAWAN_PLD_MAX_SIZE_WORDS;
		ret = ring_buf_item_get(&rx_buf.rb, &type_tmp, &val_tmp, payload_tmp,
					&payload_tmp_size);
		if (ret == -EAGAIN) {
			/* Buffer doesn't have any items -> the one we are
			 * trying to add won't fit any way. */
			return -EMSGSIZE;
		}
		LOG_DBG("Discarded ring buff elemem size %d", payload_tmp_size);
		rx_buf_avail_elem--;
		rx_buf_discarded_elem++;
		payload_tmp_size = LORAWAN_PLD_MAX_SIZE_WORDS;

		memcpy(payload_tmp, payload, len);

		ret = ring_buf_item_put(&rx_buf.rb, len, port, payload_tmp,
					SIZE_BYTES_TO_WORDS(len));
	}


	rx_buf_avail_elem++;
#else
	memcpy(rx_buffer, payload, len);

	rx_buf_avail_elem = 1;
	rx_buffer_port = port;
	rx_buffer_stored_len = len;
#endif

	return 0;
}

/**
 * @brief Return the number of downlink messages available.
 *
 * Return the number of downlink messages available in the buffer
 *
 * @retval number of downlink messages available in the buffer
 */
static int rx_buf_avail()
{
	return rx_buf_avail_elem;
}

/**
 * @brief Return the number of discarded downlink messages since last call.
 *
 * Returns the number of discarded downlink messages since this funcion has been
 * called last.
 *
 * @retval number of discarded characters
 */
static int rx_buf_discarded()
{
	int retval = rx_buf_discarded_elem;
	rx_buf_discarded_elem = 0;
	return retval;
}

static void OnMacProcessNotify(void)
{
	LoRaMacProcess();
}

static void McpsConfirm(McpsConfirm_t *mcpsConfirm)
{
	LOG_DBG("Received McpsConfirm (for McpsRequest %d)",
		mcpsConfirm->McpsRequest);

	if (mcpsConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
		LOG_ERR("McpsRequest failed : %s",
			log_strdup(eventinfo2str(mcpsConfirm->Status)));
	} else {
		LOG_DBG("McpsRequest success!");
	}

	last_mcps_confirm_status = mcpsConfirm->Status;
	k_sem_give(&mcps_confirm_sem);
}

static void McpsIndication(McpsIndication_t *mcpsIndication)
{
	int ret;
	LOG_DBG("Received McpsIndication %d", mcpsIndication->McpsIndication);

	if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
		LOG_ERR("McpsIndication failed : %s",
			log_strdup(eventinfo2str(mcpsIndication->Status)));
		return;
	}

	/* TODO: Check MCPS Indication type */
	if (mcpsIndication->RxData == true) {
		LOG_DBG("RxData set");
		if (mcpsIndication->BufferSize != 0) {
			LOG_DBG("Rx %dB Data: %s",
				mcpsIndication->BufferSize,
				log_strdup(mcpsIndication->Buffer));

			ret = rx_buf_put(mcpsIndication->Port,
				mcpsIndication->Buffer,
				mcpsIndication->BufferSize);
			if (ret == -EMSGSIZE) {
				LOG_WRN("Rx buff too small for DL size %d",
					mcpsIndication->BufferSize);
			}
		}
	}

	last_mcps_indication_status = mcpsIndication->Status;

	/* TODO: Compliance test based on FPort value*/
}

static void MlmeConfirm(MlmeConfirm_t *mlmeConfirm)
{
	MibRequestConfirm_t mibGet;

	LOG_DBG("Received MlmeConfirm (for MlmeRequest %d)",
		mlmeConfirm->MlmeRequest);

	if (mlmeConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
		LOG_ERR("MlmeConfirm failed : %s",
			log_strdup(eventinfo2str(mlmeConfirm->Status)));
		goto out_sem;
	}

	switch (mlmeConfirm->MlmeRequest) {
	case MLME_JOIN:
		mibGet.Type = MIB_DEV_ADDR;
		LoRaMacMibGetRequestConfirm(&mibGet);
		LOG_INF("Joined network! DevAddr: %08x", mibGet.Param.DevAddr);
		break;
	case MLME_LINK_CHECK:
		/* Not implemented */
		break;
	default:
		break;
	}

out_sem:
	last_mlme_confirm_status = mlmeConfirm->Status;
	k_sem_give(&mlme_confirm_sem);
}

static void MlmeIndication(MlmeIndication_t *mlmeIndication)
{
	LOG_DBG("Received MlmeIndication %d", mlmeIndication->MlmeIndication);
	last_mlme_indication_status = mlmeIndication->Status;
}

int lorawan_config(struct lorawan_mib_config *mib_config)
{
	MibRequestConfirm_t mibReq;

	mibReq.Type = MIB_NWK_KEY;
	mibReq.Param.NwkKey = mib_config->nwk_key;
	LoRaMacMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_DEV_EUI;
	mibReq.Param.DevEui = mib_config->dev_eui;
	LoRaMacMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_JOIN_EUI;
	mibReq.Param.JoinEui = mib_config->join_eui;
	LoRaMacMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_DEVICE_CLASS;
	mibReq.Param.Class = mib_config->lw_class;
	LoRaMacMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_ADR;
	mibReq.Param.AdrEnable = mib_config->adr_enable;
	LoRaMacMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_PUBLIC_NETWORK;
	mibReq.Param.EnablePublicNetwork = mib_config->pub_nw;
	LoRaMacMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_SYSTEM_MAX_RX_ERROR;
	mibReq.Param.SystemMaxRxError = mib_config->system_max_rs_error;
	LoRaMacMibSetRequestConfirm(&mibReq);

	return 0;
}

static LoRaMacStatus_t lorawan_join_otaa(enum lorawan_datarate datarate)
{
	MlmeReq_t mlmeReq;

	mlmeReq.Type = MLME_JOIN;
	mlmeReq.Req.Join.Datarate = datarate;

	return LoRaMacMlmeRequest(&mlmeReq);
}

int lorawan_join_network(enum lorawan_datarate datarate,
			 enum lorawan_act_type mode)
{
	LoRaMacStatus_t status;
	int ret = 0;

	k_mutex_lock(&lorawan_join_mutex, K_FOREVER);

	if (mode == LORAWAN_ACT_OTAA) {
		status = lorawan_join_otaa(datarate);
		if (status != LORAMAC_STATUS_OK) {
			LOG_ERR("OTAA join failed: %s",
				log_strdup(status2str(status)));
			ret = mac_status_to_errno[status];
			goto out;
		}

		LOG_DBG("Network join request sent!");

		/*
		 * We can be sure that the semaphore will be released for
		 * both success and failure cases after a specific time period.
		 * So we can use K_FOREVER and no need to check the return val.
		 */
		k_sem_take(&mlme_confirm_sem, K_FOREVER);
		if (last_mlme_confirm_status != LORAMAC_EVENT_INFO_STATUS_OK) {
			ret = mac_event_info_to_errno[last_mlme_confirm_status];
			goto out;
		}
	} else {
		ret = -EINVAL;
	}

out:
	k_mutex_unlock(&lorawan_join_mutex);
	return ret;
}

int lorawan_send(u8_t port, enum lorawan_datarate datarate, u8_t *data,
		 u8_t len, bool confirm, u8_t tries)
{
	LoRaMacStatus_t status;
	McpsReq_t mcpsReq;
	LoRaMacTxInfo_t txInfo;
	int ret = 0;
	bool empty_frame = false;

	if (data == NULL) {
		return -EINVAL;
	}

	k_mutex_lock(&lorawan_send_mutex, K_FOREVER);

	status = LoRaMacQueryTxPossible(len, &txInfo);
	if (status != LORAMAC_STATUS_OK) {
		/*
		 * If this returns false, then most likely the payload has
		 * exceeded the maximum possible length for the current region
		 * and datarate. We can't do much other than sending empty
		 * frame in order to flush MAC commands in stack and hoping the
		 * application to lower the payload size for next try.
		 */
		LOG_ERR("LoRaWAN Query Tx Possible Failed: %s",
			log_strdup(status2str(status)));
		empty_frame = true;
		mcpsReq.Type = MCPS_UNCONFIRMED;
		mcpsReq.Req.Unconfirmed.fBuffer = NULL;
		mcpsReq.Req.Unconfirmed.fBufferSize = 0;
		mcpsReq.Req.Unconfirmed.Datarate = DR_0;
	} else {
		if (confirm == false) {
			mcpsReq.Type = MCPS_UNCONFIRMED;
			mcpsReq.Req.Unconfirmed.fPort = port;
			mcpsReq.Req.Unconfirmed.fBuffer = data;
			mcpsReq.Req.Unconfirmed.fBufferSize = len;
			mcpsReq.Req.Unconfirmed.Datarate = datarate;
		} else {
			mcpsReq.Type = MCPS_CONFIRMED;
			mcpsReq.Req.Confirmed.fPort = port;
			mcpsReq.Req.Confirmed.fBuffer = data;
			mcpsReq.Req.Confirmed.fBufferSize = len;
			mcpsReq.Req.Confirmed.NbTrials = tries;
			mcpsReq.Req.Confirmed.Datarate = datarate;
		}
	}

	status = LoRaMacMcpsRequest(&mcpsReq);
	if (status != LORAMAC_STATUS_OK) {
		LOG_ERR("LoRaWAN Send failed: %s",
			log_strdup(status2str(status)));
		ret = mac_status_to_errno[status];
		goto out;
	}

	/*
	 * Indicate the application that the current packet is not sent and
	 * it has to resend the packet
	 */
	if (empty_frame) {
		ret = -EAGAIN;
		goto out;
	}

	/* Wait for send confirmation */
	if (confirm) {
		/*
		 * We can be sure that the semaphore will be released for
		 * both success and failure cases after a specific time period.
		 * So we can use K_FOREVER and no need to check the return val.
		 */
		k_sem_take(&mcps_confirm_sem, K_FOREVER);

		if (last_mcps_confirm_status != LORAMAC_EVENT_INFO_STATUS_OK) {
			ret = mac_event_info_to_errno[last_mcps_confirm_status];
		}
	}

out:
	k_mutex_unlock(&lorawan_send_mutex);
	return ret;
}

int lorawan_receive_available()
{
	return rx_buf_avail();
}

int lorawan_receive_read(u8_t *port, u8_t *data, u8_t len)
{
	return rx_buf_get(port, data, len);
}

int lorawan_receive_discarded()
{
	return rx_buf_discarded();
}

static int lorawan_init(struct device *dev)
{
	LoRaMacStatus_t status;

	macPrimitives.MacMcpsConfirm = McpsConfirm;
	macPrimitives.MacMcpsIndication = McpsIndication;
	macPrimitives.MacMlmeConfirm = MlmeConfirm;
	macPrimitives.MacMlmeIndication = MlmeIndication;
	macCallbacks.GetBatteryLevel = NULL;
	macCallbacks.GetTemperatureLevel = NULL;
	macCallbacks.NvmContextChange = NULL;
	macCallbacks.MacProcessNotify = OnMacProcessNotify;

	status = LoRaMacInitialization(&macPrimitives, &macCallbacks,
				       LORAWAN_REGION);
	if (status != LORAMAC_STATUS_OK) {
		LOG_ERR("LoRaMacInitialization failed: %s",
			log_strdup(status2str(status)));
		return -EINVAL;
	}


#if CONFIG_LORAWAN_USE_RX_RING_BUFFER
	ring_buf_init(&rx_buf.rb, RX_RING_BUF_SIZE_WORDS, rx_buf.buffer);
#endif

	rx_buf_avail_elem = 0;
	rx_buf_discarded_elem = 0;

	LoRaMacStart();

	LOG_DBG("LoRaMAC Initialized");

	return 0;
}

SYS_INIT(lorawan_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
