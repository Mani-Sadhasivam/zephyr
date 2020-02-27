/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>
#include <errno.h>
#include <lorawan.h>
#include <stddef.h>
#include <string.h>

#include <zephyr.h>
#include <soc.h>
#include <LoRaMac.h>

#ifdef CONFIG_LORAMAC_REGION_AS923
	#define CONFIG_LORAWAN_REGION LORAMAC_REGION_AS923
#elif CONFIG_LORAMAC_REGION_AU915
	#define CONFIG_LORAWAN_REGION LORAMAC_REGION_AU915
#elif CONFIG_LORAMAC_REGION_CN470
	#define CONFIG_LORAWAN_REGION LORAMAC_REGION_CN470
#elif CONFIG_LORAMAC_REGION_CN779
	#define CONFIG_LORAWAN_REGION LORAMAC_REGION_CN779
#elif CONFIG_LORAMAC_REGION_EU433
	#define CONFIG_LORAWAN_REGION LORAMAC_REGION_EU433
#elif CONFIG_LORAMAC_REGION_EU868
	#define CONFIG_LORAWAN_REGION LORAMAC_REGION_EU868
#elif CONFIG_LORAMAC_REGION_KR920
	#define CONFIG_LORAWAN_REGION LORAMAC_REGION_KR920
#elif CONFIG_LORAMAC_REGION_IN865
	#define CONFIG_LORAWAN_REGION LORAMAC_REGION_IN865
#elif CONFIG_LORAMAC_REGION_US915
	#define CONFIG_LORAWAN_REGION LORAMAC_REGION_US915
#elif CONFIG_LORAMAC_REGION_RU864
	#define CONFIG_LORAWAN_REGION LORAMAC_REGION_RU864
#elif
	#error "Atleast one LoRaWAN region should be selected"
#endif

#define LOG_LEVEL CONFIG_LORAWAN_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(lorawan);

K_SEM_DEFINE(lorawan_config_sem, 0, 1);

/* MAC status strings */
const char *to_status_str[] = {
    "OK",                            /* LORAMAC_STATUS_OK */
    "Busy",                          /* LORAMAC_STATUS_BUSY */
    "Service unknown",               /* LORAMAC_STATUS_SERVICE_UNKNOWN */
    "Parameter invalid",             /* LORAMAC_STATUS_PARAMETER_INVALID */
    "Frequency invalid",             /* LORAMAC_STATUS_FREQUENCY_INVALID */
    "Datarate invalid",              /* LORAMAC_STATUS_DATARATE_INVALID */
    "Frequency or datarate invalid", /* LORAMAC_STATUS_FREQ_AND_DR_INVALID */
    "No network joined",             /* LORAMAC_STATUS_NO_NETWORK_JOINED */
    "Length error",                  /* LORAMAC_STATUS_LENGTH_ERROR */
    "Region not supported",          /* LORAMAC_STATUS_REGION_NOT_SUPPORTED */
    "Skipped APP data",              /* LORAMAC_STATUS_SKIPPED_APP_DATA */
    "Duty-cycle restricted",         /* LORAMAC_STATUS_DUTYCYCLE_RESTRICTED */
    "No channel found",              /* LORAMAC_STATUS_NO_CHANNEL_FOUND */
    "No free channel found",         /* LORAMAC_STATUS_NO_FREE_CHANNEL_FOUND */
    "Busy beacon reserved time",     /* LORAMAC_STATUS_BUSY_BEACON_RESERVED_TIME */
    "Busy ping-slot window time",    /* LORAMAC_STATUS_BUSY_PING_SLOT_WINDOW_TIME */
    "Busy uplink collision",         /* LORAMAC_STATUS_BUSY_UPLINK_COLLISION */
    "Crypto error",                  /* LORAMAC_STATUS_CRYPTO_ERROR */
    "FCnt handler error",            /* LORAMAC_STATUS_FCNT_HANDLER_ERROR */
    "MAC command error",             /* LORAMAC_STATUS_MAC_COMMAD_ERROR */
    "ClassB error",                  /* LORAMAC_STATUS_CLASS_B_ERROR */
    "Confirm queue error",           /* LORAMAC_STATUS_CONFIRM_QUEUE_ERROR */
    "Multicast group undefined",     /* LORAMAC_STATUS_MC_GROUP_UNDEFINED */
    "Unknown error",                 /* LORAMAC_STATUS_ERROR */
};

/* MAC event info status strings */
const char *to_event_info_status_str[] = { 
    "OK",                            /* LORAMAC_EVENT_INFO_STATUS_OK */
    "Error",                         /* LORAMAC_EVENT_INFO_STATUS_ERROR */
    "Tx timeout",                    /* LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT */
    "Rx 1 timeout",                  /* LORAMAC_EVENT_INFO_STATUS_RX1_TIMEOUT */
    "Rx 2 timeout",                  /* LORAMAC_EVENT_INFO_STATUS_RX2_TIMEOUT */
    "Rx1 error",                     /* LORAMAC_EVENT_INFO_STATUS_RX1_ERROR */
    "Rx2 error",                     /* LORAMAC_EVENT_INFO_STATUS_RX2_ERROR */
    "Join failed",                   /* LORAMAC_EVENT_INFO_STATUS_JOIN_FAIL */
    "Downlink repeated",             /* LORAMAC_EVENT_INFO_STATUS_DOWNLINK_REPEATED */
    "Tx DR payload size error",      /* LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR */
    "Downlink too many frames loss", /* LORAMAC_EVENT_INFO_STATUS_DOWNLINK_TOO_MANY_FRAMES_LOSS */
    "Address fail",                  /* LORAMAC_EVENT_INFO_STATUS_ADDRESS_FAIL */
    "MIC fail",                      /* LORAMAC_EVENT_INFO_STATUS_MIC_FAIL */
    "Multicast fail",                /* LORAMAC_EVENT_INFO_STATUS_MULTICAST_FAIL */
    "Beacon locked",                 /* LORAMAC_EVENT_INFO_STATUS_BEACON_LOCKED */
    "Beacon lost",                   /* LORAMAC_EVENT_INFO_STATUS_BEACON_LOST */
    "Beacon not found"               /* LORAMAC_EVENT_INFO_STATUS_BEACON_NOT_FOUND */
};

static LoRaMacPrimitives_t macPrimitives;
static LoRaMacCallback_t macCallbacks;

void OnMacProcessNotify(void)
{
	LoRaMacProcess();
}

static void McpsConfirm(McpsConfirm_t *mcpsConfirm)
{
	if (mcpsConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
		LOG_ERR("McpsRequest failed : %s",
			log_strdup(to_event_info_status_str[mcpsConfirm->Status]));
	} else {
		LOG_DBG("McpsRequest success!");
	}
}

static void McpsIndication(McpsIndication_t *mcpsIndication)
{
	if (mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
		LOG_ERR("McpsIndication failed : %s",
			log_strdup(to_event_info_status_str[mcpsIndication->Status]));
		return;
	}

	/* TODO: Check MCPS Indication type */
	if (mcpsIndication->RxData == true) {
		if (mcpsIndication->BufferSize != 0) {
			LOG_DBG("Rx Data: %s", log_strdup(mcpsIndication->Buffer));
		}
	}

	/* TODO: Compliance test based on FPort value*/
}

static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
	MibRequestConfirm_t mibGet;

	if (mlmeConfirm->Status != LORAMAC_EVENT_INFO_STATUS_OK) {
		LOG_ERR("McpsIndication failed : %s",
			log_strdup(to_event_info_status_str[mlmeConfirm->Status]));
		return;
	}

	switch (mlmeConfirm->MlmeRequest) {
	case MLME_JOIN:
		mibGet.Type = MIB_DEV_ADDR;
		LoRaMacMibGetRequestConfirm(&mibGet);
		k_sem_give(&lorawan_config_sem);
		LOG_DBG("Joined network! DevAddr: %08x", mibGet.Param.DevAddr);
		break;
	case MLME_LINK_CHECK:
		/* Not implemented */
		break;
	default:
		break;
	}
}

static void MlmeIndication( MlmeIndication_t *mlmeIndication )
{
	LOG_DBG("MlmeIndication!");
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

	/* LoRaWAN is by default a public network */
	mibReq.Type = MIB_PUBLIC_NETWORK;
	mibReq.Param.EnablePublicNetwork = true;
	LoRaMacMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_JOIN_ACCEPT_DELAY_1;
	mibReq.Param.JoinAcceptDelay1 = mib_config->join_acc_delay1;
	LoRaMacMibSetRequestConfirm(&mibReq);

	mibReq.Type = MIB_JOIN_ACCEPT_DELAY_2;
	mibReq.Param.JoinAcceptDelay2 = mib_config->join_acc_delay2;
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

int lorawan_join_network(enum lorawan_datarate datarate, enum lorawan_act_type mode)
{
	LoRaMacStatus_t status;

	if (mode == LORAWAN_ACT_OTAA) {
		status = lorawan_join_otaa(datarate);
		if (status != LORAMAC_STATUS_OK) {
			LOG_ERR("OTAA join failed: %s",
				log_strdup(to_status_str[status]));
			return -EINVAL;	
		}

		LOG_DBG("Network join request sent!");

		k_sem_take(&lorawan_config_sem, K_FOREVER);
	}

	return 0;
}

int lorawan_send(u8_t port, enum lorawan_datarate datarate, u8_t *data,
		 u8_t len, bool confirm, u8_t tries)
{
	LoRaMacStatus_t status;
	McpsReq_t mcpsReq;
	LoRaMacTxInfo_t txInfo;

	if (data == NULL) {
		return -EINVAL;
	}

	/* TODO: Do we need to make a copy here? */
	//memcpy(data, data, len);

	if( LoRaMacQueryTxPossible( len, &txInfo ) != LORAMAC_STATUS_OK ) {
		/* Send empty frame in order to flush MAC commands */
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
			log_strdup(to_status_str[status]));
		return -EINVAL;
	}

	return 0;
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
				       CONFIG_LORAWAN_REGION);
	if (status != LORAMAC_STATUS_OK) {
		LOG_ERR("LoRaMacInitialization failed: %s",
			log_strdup(to_status_str[status]));
		return -EINVAL;
	}

	LoRaMacStart();

	LOG_DBG("LoRaMAC Initialized");

	return 0;
}

SYS_INIT(lorawan_init, APPLICATION, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);
