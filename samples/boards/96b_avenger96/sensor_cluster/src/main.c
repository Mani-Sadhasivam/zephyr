/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <string.h>
#include <sys/util.h>

static struct sensor_value accel_x, accel_y, accel_z;
static struct sensor_value gyro_x, gyro_y, gyro_z;
static struct sensor_value temp;
static struct device *mpu9250;
static struct device *bmp180;

static bool mpu9250_available = true;
static bool bmp180_available = true;

void init_sensors();

#include <ipm.h>
#include <openamp/open_amp.h>
#include <metal/device.h>
#include <resource_table.h>

extern int __OPENAMP_region_start__[];   /* defined by linker script */
extern int __OPENAMP_region_end__[];  /* defined by linker script */

#define RPMSG_CHAN_NAME	"rpmsg-tty-channel"
#define SHM_DEVICE_NAME	"shm"

/* constant derivated from linker symbols */
#define SHM_START_ADDR	DT_IPC_SHM_BASE_ADDRESS
#define SHM_SIZE (DT_IPC_SHM_SIZE * 1024)

#define APP_TASK_STACK_SIZE (1024)
K_THREAD_STACK_DEFINE(thread_stack, APP_TASK_STACK_SIZE);
static struct k_thread thread_data;

static struct device *ipm_handle;

static metal_phys_addr_t shm_physmap = SHM_START_ADDR;

struct metal_device shm_device = {
	.name = SHM_DEVICE_NAME,
	.num_regions = 2,
	.regions = {
		{.virt = NULL}, /* shared memory */
		{.virt = NULL}, /* rsc_table memory */
	},
	.node = { NULL },
	.irq_num = 0,
	.irq_info = NULL
};

static struct metal_io_region *shm_io;
static struct rpmsg_virtio_shm_pool shpool;

static struct metal_io_region *rsc_io;
static struct rpmsg_virtio_device rvdev;

static void *rsc_table;

static char rcv_msg[20];
static unsigned int rcv_len;
static struct rpmsg_endpoint rcv_ept;

static K_SEM_DEFINE(data_sem, 0, 1);
static K_SEM_DEFINE(data_rx_sem, 0, 1);

static bool first_data = true;

static void platform_ipm_callback(void *context, u32_t id, volatile void *data)
{
	if (first_data)
		k_sem_give(&data_sem);
}

static int rpmsg_recv_callback(struct rpmsg_endpoint *ept, void *data,
			       size_t len, uint32_t src, void *priv)
{
	memcpy(rcv_msg, data, len);
	rcv_len = len;
	k_sem_give(&data_rx_sem);

	return RPMSG_SUCCESS;
}

static void receive_message(unsigned char **msg, unsigned int *len)
{
	while (k_sem_take(&data_rx_sem, K_NO_WAIT) != 0) {
		int status;
		if (first_data) {
			status = k_sem_take(&data_sem, K_FOREVER);
			first_data = false;
		} else {
			status = 0;
		}

		if (status == 0) {
			rproc_virtio_notified(rvdev.vdev, VRING1_ID);
		}
	}
	*len = rcv_len;
	*msg = rcv_msg;
}
static void new_service_cb(struct rpmsg_device *rdev, const char *name,
			   uint32_t src)
{
	printf("%s: unexpected ns service receive for name %s\n",
		__func__, name);
}

int mailbox_notify(void *priv, uint32_t id)
{
	ARG_UNUSED(priv);

	ipm_send(ipm_handle, 0, id, NULL, 0);

	return 0;
}
int platform_init(void)
{
	void *rsc_tab_addr;
	int rsc_size;
	struct metal_device *device;
	struct metal_init_params metal_params = METAL_INIT_DEFAULTS;
	int status;

	status = metal_init(&metal_params);
	if (status) {
		printf("Couldn't register shared memory: %d\n", status);
		return -1;
	}

	status = metal_register_generic_device(&shm_device);
	if (status) {
		printf("Couldn't register shared memory: %d\n", status);
		return -1;
	}

	status = metal_device_open("generic", SHM_DEVICE_NAME, &device);
	if (status) {
		printf("metal_device_open failed: %d\n", status);
		return -1;
	}

	/* declare shared memory region */
	metal_io_init(&device->regions[0], (void *)SHM_START_ADDR, &shm_physmap,
		      SHM_SIZE, -1, 0, NULL);

	shm_io = metal_device_io_region(device, 0);
	if (!shm_io) {
		printf("Failed to get shm_io region\n");
		return -1;
	}

	/* declare resource table region */
	rsc_table_get(&rsc_tab_addr, &rsc_size);
	rsc_table = (struct st_resource_table *)rsc_tab_addr;

	metal_io_init(&device->regions[1], rsc_table,
		      (metal_phys_addr_t *)rsc_table, rsc_size, -1, 0, NULL);

	rsc_io = metal_device_io_region(device, 1);
	if (!shm_io) {
		printf("Failed to get rsc_io region\n");
		return -1;
	}

	/* setup IPM */
	ipm_handle = device_get_binding(DT_IPM_DEV);
	if (!ipm_handle) {
		printf("Failed to find ipm device\n");
		return -1;
	}

	ipm_register_callback(ipm_handle, platform_ipm_callback, NULL);

	status = ipm_set_enabled(ipm_handle, 1);
	if (status) {
		printf("ipm_set_enabled failed\n");
		return -1;
	}

	return 0;
}

static void cleanup_system(void)
{
	ipm_set_enabled(ipm_handle, 0);
	rpmsg_deinit_vdev(&rvdev);
	metal_finish();
}

struct  rpmsg_device *
platform_create_rpmsg_vdev(unsigned int vdev_index,
			   unsigned int role,
			   void (*rst_cb)(struct virtio_device *vdev),
			   rpmsg_ns_bind_cb ns_cb)
{
	struct fw_rsc_vdev_vring *vring_rsc;
	struct virtio_device *vdev;
	int ret;

	vdev = rproc_virtio_create_vdev(VIRTIO_DEV_SLAVE, VDEV_ID,
					rsc_table_to_vdev(rsc_table),
					rsc_io, NULL, mailbox_notify, NULL);

	if (!vdev) {
		printf("failed to create vdev\r\n");
		return NULL;
	}

	/* wait master rpmsg init completion */
	rproc_virtio_wait_remote_ready(vdev);

	vring_rsc = rsc_table_get_vring0(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 0, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		printf("failed to init vring 0\r\n");
		goto failed;
	}

	vring_rsc = rsc_table_get_vring1(rsc_table);
	ret = rproc_virtio_init_vring(vdev, 1, vring_rsc->notifyid,
				      (void *)vring_rsc->da, rsc_io,
				      vring_rsc->num, vring_rsc->align);
	if (ret) {
		printf("failed to init vring 1\r\n");
		goto failed;
	}

	rpmsg_virtio_init_shm_pool(&shpool, NULL, SHM_SIZE);
	ret =  rpmsg_init_vdev(&rvdev, vdev, ns_cb, shm_io, &shpool);

	if (ret) {
		printf("failed rpmsg_init_vdev\r\n");
		goto failed;
	}

	return rpmsg_virtio_get_rpmsg_device(&rvdev);

failed:
	rproc_virtio_remove_vdev(vdev);

	return NULL;
}

void remote_send(char *msg, int len) {
		rpmsg_send(&rcv_ept, msg, len);
}

void app_task(void *arg1, void *arg2, void *arg3)
{
	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);
	struct rpmsg_device *rpdev;
	unsigned char *msg;
	int len;
	int ret = 0;
	char message[128];

	printk("Avenger96 Sensor Cluster Demo Started\n");

	/* Initialize platform */
	ret = platform_init();
	if (ret) {
		printf("Failed to initialize platform\n");
		ret = -1;
		goto task_end;
	}

	rpdev = platform_create_rpmsg_vdev(0, VIRTIO_DEV_SLAVE, NULL,
					   new_service_cb);
	if (!rpdev) {
		printf("Failed to create rpmsg virtio device\n");
		ret = -1;
		goto task_end;
	}

	ret = rpmsg_create_ept(&rcv_ept, rpdev, RPMSG_CHAN_NAME,
			       RPMSG_ADDR_ANY, RPMSG_ADDR_ANY,
			       rpmsg_recv_callback, NULL);
	if (ret != 0)
		printf("error while creating endpoint(%d)\n", ret);

	ipm_send(ipm_handle, 0, 0, NULL, 0);

	init_sensors();

	receive_message(&msg, &len);

	while (1) {
		/* Get sensor samples */
		if (mpu9250_available && sensor_sample_fetch(mpu9250) < 0) {
			printf("MPU9250 Sensor sample update error\n");
		}
		if (bmp180_available && sensor_sample_fetch(bmp180) < 0) {
			printf("BMP180 Sensor sample update error\n");
		}

		/* Get sensor data */
		if (mpu9250_available) {
			sensor_channel_get(mpu9250, SENSOR_CHAN_ACCEL_X, &accel_x);
			sensor_channel_get(mpu9250, SENSOR_CHAN_ACCEL_Y, &accel_y);
			sensor_channel_get(mpu9250, SENSOR_CHAN_ACCEL_Z, &accel_z);
			sensor_channel_get(mpu9250, SENSOR_CHAN_GYRO_X, &gyro_x);
			sensor_channel_get(mpu9250, SENSOR_CHAN_GYRO_Y, &gyro_y);
			sensor_channel_get(mpu9250, SENSOR_CHAN_GYRO_Z, &gyro_z);
		}
		if (bmp180_available) {
			sensor_channel_get(bmp180, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		}

		/* Display sensor data */
		len = snprintf(message, sizeof(message), "96Boards Avenger96 Sensor Cluster:\n");
		remote_send(message, len);

		if (mpu9250_available) {
			/* ACCEL */
			len = snprintf(message, sizeof(message),
				"MPU9250: Accel ( x y z ) = ( %f  %f  %f )\n",
				sensor_value_to_double(&accel_x),
				sensor_value_to_double(&accel_y),
				sensor_value_to_double(&accel_z));
			remote_send(message, len);

			/* GYRO */
			len = snprintf(message, sizeof(message),
				"MPU9250: Gyro ( x y z ) = ( %f  %f  %f )\n",
				sensor_value_to_double(&gyro_x),
				sensor_value_to_double(&gyro_y),
				sensor_value_to_double(&gyro_z));
			remote_send(message, len);
		}

		if (bmp180_available) {
			/* bmp180 temperature */
			len = snprintf(message, sizeof(message),
					"BMP180: Temperature: %.1f C\n\n",
					sensor_value_to_double(&temp));
			remote_send(message, len);
		}

		k_sleep(2000);
	}

	rpmsg_destroy_ept(&rcv_ept);

task_end:
	cleanup_system();

	printk("Avenger96 Sensor Cluster Demo Ended\n");
}

void init_sensors() {
	/* search in devicetree if sensors are referenced */
	mpu9250 = device_get_binding(CONFIG_MPU6050_NAME);
	bmp180 = device_get_binding(DT_INST_0_BOSCH_BME280_LABEL);

	if (mpu9250 == NULL) {
		printf("Could not get MPU9250 device\n");
		mpu9250_available = false;
	}
	if (bmp180 == NULL) {
		printf("Could not get BMP180 device\n");
		bmp180_available = false;
	}
}

void main(void)
{
	/* openAMP */
	printk("Starting application thread!\n");
	k_thread_create(&thread_data, thread_stack, APP_TASK_STACK_SIZE,
			(k_thread_entry_t)app_task,
			NULL, NULL, NULL, K_PRIO_COOP(7), 0, 0);
}
