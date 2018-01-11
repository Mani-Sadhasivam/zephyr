/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <misc/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>

#define CID_INTEL			0x0002
#define ID_TEMP_CELSIUS			0x2A1F

#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS	BT_MESH_MODEL_OP_2(0x82, 0x04)
#define BT_MESH_MODEL_OP_SENSOR_STATUS		BT_MESH_MODEL_OP_1(0x52)
#define BT_MESH_MODEL_OP_SENSOR_GET		BT_MESH_MODEL_OP_2(0x82, 0x31)

static struct k_work temp_work;
static struct k_timer temp_timer;

static u8_t btn_status = 0;

static u16_t node_addr;

static struct bt_mesh_cfg_srv cfg_srv = {
        .relay = BT_MESH_RELAY_DISABLED,
        .beacon = BT_MESH_BEACON_ENABLED,
        .frnd = BT_MESH_FRIEND_NOT_SUPPORTED,
        .gatt_proxy = BT_MESH_GATT_PROXY_ENABLED,
        .default_ttl = 7,

        /* 3 transmissions with 20ms interval */
        .net_transmit = BT_MESH_TRANSMIT(2, 20),
        .relay_retransmit = BT_MESH_TRANSMIT(2, 20),
};

static struct bt_mesh_model_pub temp_cli_pub = {
	.msg = NET_BUF_SIMPLE(2),
};

static struct bt_mesh_model_pub gen_cli_pub = {
	.msg = NET_BUF_SIMPLE(2),
};

static void temp_cli_get(struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
}

static void temp_cli_status(struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
	u16_t temp, temp_id, co2, co2_id;
	u8_t node_id;
	
	temp_id = net_buf_simple_pull_le16(buf);
	temp = net_buf_simple_pull_le16(buf);
	co2_id = net_buf_simple_pull_le16(buf);
	co2 = net_buf_simple_pull_le16(buf);
	node_id = net_buf_simple_pull_u8(buf);

	if (btn_status) {
		printk("%d,%d,%d,%d\n", node_id, temp, co2, btn_status);
		btn_status = 0;
	} else {
		printk("%d,%d,%d,%d\n", node_id, temp, co2, btn_status);
	}
}

static void gen_cli_status(struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
	btn_status = 1;
}

/* Sensor client model Opcode */
static const struct bt_mesh_model_op temp_cli_op[] = {
	/* Opcode, message length, message handler */
        { BT_MESH_MODEL_OP_SENSOR_GET, 2, temp_cli_get },
        { BT_MESH_MODEL_OP_SENSOR_STATUS, 2, temp_cli_status },
        BT_MESH_MODEL_OP_END,
};

/* Generic client model Opcode */
static const struct bt_mesh_model_op gen_cli_op[] = {
	/* Opcode, message length, message handler */
        { BT_MESH_MODEL_OP_GEN_ONOFF_STATUS, 1, gen_cli_status },
        BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model root_models[] = {
        /* Mandatory Configuration Server model. Should be the first model
         * of root element */
        BT_MESH_MODEL_CFG_SRV(&cfg_srv),
        BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_CLI, temp_cli_op,
                      &temp_cli_pub, NULL),
        BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_CLI, gen_cli_op,
                      &gen_cli_pub, NULL),
};

static struct bt_mesh_elem elements[] = {
        BT_MESH_ELEM(0, root_models, BT_MESH_MODEL_NONE),
};

/* Node composition data used to configure a node while provisioning */
static const struct bt_mesh_comp comp = {
        .cid = CID_INTEL,
        .elem = elements,
        .elem_count = ARRAY_SIZE(elements),
};

static int output_number(bt_mesh_output_action_t action, uint32_t number)
{
        printk("OOB Number: %u\n", number);
        
	return 0;
}

static void prov_complete(u16_t net_idx, u16_t addr)
{
        printk("Provisioning completed!\n");
	printk("Net ID: %u\n", net_idx);
	printk("Unicast addr: 0x%04x\n", addr);
	
	node_addr = addr;
}

/* UUID for identifying the unprovisioned node */
static const uint8_t dev_uuid[16] = { 0xdd, 0xdd };

/* Only displaying the number while provisioning is supported */
static const struct bt_mesh_prov prov = {
        .uuid = dev_uuid,
        .output_size = 4,
        .output_actions = BT_MESH_DISPLAY_NUMBER,
        .output_number = output_number,
        .complete = prov_complete,
};

static void bt_ready(int err)
{
	int ret;

        if (err) {
                printk("Bluetooth init failed (err %d)\n", err);
                return;
        }

        printk("Bluetooth initialized\n");

        ret = bt_mesh_init(&prov, &comp);
        if (ret) {
                printk("Initializing mesh failed (err %d)\n", ret);
                return;
        }

	bt_mesh_prov_enable(BT_MESH_PROV_GATT | BT_MESH_PROV_ADV);
	
        printk("Mesh initialized\n");
}

static void temp_timer_thread(struct k_timer *work)
{
	k_work_submit(&temp_work);
}

/* send unsolicited request to get sensor readings */
void temp_work_thread(struct k_work *work)
{
	struct bt_mesh_model *model = &root_models[1];
       	struct net_buf_simple *msg = model->pub->msg;
	int ret;

	if (node_addr == BT_MESH_ADDR_UNASSIGNED)
		return;

	/* sensor status */
        bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENSOR_GET);
	net_buf_simple_add_le16(msg, ID_TEMP_CELSIUS);

	ret = bt_mesh_model_publish(model);
	if (ret) {
		printk("ERR: Unable to send sensor status get request: %d\n", ret);
		return;
	}
}

void main(void)
{
        int ret;

	/* Initialize temp work thread */
	k_work_init(&temp_work, temp_work_thread);

	/* Initialize temp timer thread */
	k_timer_init(&temp_timer, temp_timer_thread, NULL);
        
	printk("Initializing...\n");

        /* Initialize the Bluetooth Subsystem */
        ret = bt_enable(bt_ready);
        if (ret) {
                printk("Bluetooth init failed (err %d)\n", ret);
        }

	/* Start the timer at 5 seconds interval */
	k_timer_start(&temp_timer, K_SECONDS(50), K_SECONDS(5));
}
