/* main.c - Application main entry point */

/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <gpio.h>
#include <sensor.h>
#include <misc/printk.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh.h>

#define CID_INTEL			0x0002
#define NODE_ID				1

/* Propery ID of sensors */
#define ID_TEMP_CELSIUS			0x2A1F
/* Use Analog Output ID for reporting Co2 level as it is undefined in GATT spec */
#define ID_CO2_PPM			0x2A59

#define BT_MESH_MODEL_OP_SENSOR_GET     BT_MESH_MODEL_OP_2(0x82, 0x31)
#define BT_MESH_MODEL_OP_GEN_ONOFF_STATUS  BT_MESH_MODEL_OP_2(0x82, 0x04)
#define BT_MESH_MODEL_OP_SENSOR_STATUS  BT_MESH_MODEL_OP_1(0x52)

/* Emergency button */
#define E_BUTTON_PORT			"GPIOC"
#define E_BUTTON			8

static struct device *ccs811, *tmp102, *gpio;

static struct gpio_callback button_cb;

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

static struct bt_mesh_model_pub sensor_srv_pub = {
	.msg = NET_BUF_SIMPLE(2 + 2 + 2 + 2 + 1),
};

static struct bt_mesh_model_pub gen_srv_pub = {
	.msg = NET_BUF_SIMPLE(2 + 1),
};

static struct bt_mesh_model_pub health_pub = {
        .msg  = BT_MESH_HEALTH_FAULT_MSG(0),
};

/*
 * Health Server Declaration
 */

static struct bt_mesh_health_srv health_srv = {
};

static void gen_onoff_status(struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
}
static void sensor_srv_status(struct bt_mesh_model *model,
                               struct bt_mesh_msg_ctx *ctx,
                               struct net_buf_simple *buf)
{
       	struct net_buf_simple *msg = model->pub->msg;
	struct sensor_value tmp102_value;
	struct sensor_value ccs811_value;
	int ret;

	printk("Sensor Status Get request received\n");

	/* get temperature value from sensor */
	sensor_sample_fetch(tmp102);
	sensor_channel_get(tmp102, SENSOR_CHAN_TEMP, &tmp102_value);
	printk("Temp: %d\n", tmp102_value.val1);

	/* get co2 value from sensor */
	sensor_sample_fetch(ccs811);
	sensor_channel_get(ccs811, SENSOR_CHAN_CO2, &ccs811_value);
	printk("Co2: %d\n", ccs811_value.val1);
	
	/* begin sending sensor status */
        bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_SENSOR_STATUS);
	/* id: termperature in celsius */
	net_buf_simple_add_le16(msg, ID_TEMP_CELSIUS);
	/* temperature value in celsius */
	net_buf_simple_add_le16(msg, tmp102_value.val1);
	/* id: co2 level in ppm */
	net_buf_simple_add_le16(msg, ID_CO2_PPM);
	/* co2 level in ppm */
	net_buf_simple_add_le16(msg, ccs811_value.val1);
	/* send the node ID */
	net_buf_simple_add_u8(msg, NODE_ID);

	ret = bt_mesh_model_publish(model);
	if (ret) {
		printk("ERR: Unable to publish sensor status: %d\n", ret);
		return;
	}
 
	printk("Sensor status sent with OpCode 0x%08x\n", BT_MESH_MODEL_OP_SENSOR_STATUS);
}

/* Sensor server model Opcode */
static const struct bt_mesh_model_op sensor_srv_op[] = {
	/* Opcode, message length, message handler */
        { BT_MESH_MODEL_OP_SENSOR_GET, 2, sensor_srv_status },
        BT_MESH_MODEL_OP_END,
};

/* Generic server model Opcode */
static const struct bt_mesh_model_op gen_srv_op[] = {
	/* Opcode, message length, message handler */
        { BT_MESH_MODEL_OP_GEN_ONOFF_STATUS, 2, gen_onoff_status },
        BT_MESH_MODEL_OP_END,
};

static struct bt_mesh_model root_models[] = {
	/* Mandatory Configuration Server model. Should be the first model
	 * of root element */
        BT_MESH_MODEL_CFG_SRV(&cfg_srv),
	BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_SENSOR_SRV, sensor_srv_op,
                      &sensor_srv_pub, NULL),
	BT_MESH_MODEL(BT_MESH_MODEL_ID_GEN_ONOFF_SRV, gen_srv_op,
                      &gen_srv_pub, NULL),
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

static void button_pressed(struct device *dev, struct gpio_callback *cb,
                           uint32_t pins)
{
	struct bt_mesh_model *model = &root_models[3];
	struct net_buf_simple *msg = model->pub->msg;
	int ret;

        if (node_addr == BT_MESH_ADDR_UNASSIGNED)
                return;
	
        bt_mesh_model_msg_init(msg, BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
        net_buf_simple_add_u8(msg, 0x01);

        ret = bt_mesh_model_publish(model);
        if (ret) {
                printk("ERR: Unable to publish button status: %d\n", ret);
                return;
        }

        printk("Emergency button status sent with OpCode 0x%08x\n", BT_MESH_MODEL_OP_GEN_ONOFF_STATUS);
}	

void main(void)
{
        int ret;
	struct sensor_value attr;

	printk("Initializing...\n");

	ccs811 = device_get_binding("CCS811");
	if (!ccs811) {
		printk("Failed to get CCS811 binding");
		return;
	}

	/* TMP112 driver is compatible with TMP102 */
	tmp102 = device_get_binding("TMP112");
	if (!tmp102) {
		printk("Failed to get TMP102 binding");
		return;
	}

        attr.val1 = 150;
        attr.val2 = 0;
        ret = sensor_attr_set(tmp102, SENSOR_CHAN_TEMP,
                              SENSOR_ATTR_FULL_SCALE, &attr);
        if (ret) {
                printk("sensor_attr_set failed ret %d\n", ret);
                return;
        }

        attr.val1 = 8;
        attr.val2 = 0;
        ret = sensor_attr_set(tmp102, SENSOR_CHAN_TEMP,
                              SENSOR_ATTR_SAMPLING_FREQUENCY, &attr);
        if (ret) {
                printk("sensor_attr_set failed ret %d\n", ret);
                return;
        }


	gpio = device_get_binding(E_BUTTON_PORT);
	if (!gpio) {
		printk("Failed to get GPIO binding");
		return;
	}

        gpio_pin_configure(gpio, E_BUTTON,
                          (GPIO_DIR_IN | GPIO_INT | GPIO_INT_EDGE |
                           GPIO_INT_ACTIVE_LOW | GPIO_PUD_PULL_UP));
        gpio_init_callback(&button_cb, button_pressed, BIT(E_BUTTON));
        gpio_add_callback(gpio, &button_cb);
        gpio_pin_enable_callback(gpio, E_BUTTON);

        /* Initialize the Bluetooth Subsystem */
        ret = bt_enable(bt_ready);
        if (ret) {
                printk("Bluetooth init failed (err %d)\n", ret);
        }
}

