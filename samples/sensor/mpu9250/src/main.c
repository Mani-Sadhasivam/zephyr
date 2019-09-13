/*
 * Copyright (c) 2018 Linaro Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <stdio.h>
#include <drivers/sensor.h>

static void do_main(struct device *dev)
{
	static struct sensor_value accel_x, accel_y, accel_z;
	static struct sensor_value gyro_x, gyro_y, gyro_z;

	while (1) {
		sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
		sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
		sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);

		printf("Accel ( x y z ) = ( %f  %f  %f )\n", sensor_value_to_double(&accel_x),
						       sensor_value_to_double(&accel_y),
						       sensor_value_to_double(&accel_z));
		printf("Gyro ( x y z ) = ( %f  %f  %f )\n", sensor_value_to_double(&gyro_x),
						       sensor_value_to_double(&gyro_y),
						       sensor_value_to_double(&gyro_z));

		k_sleep(2000);
	}
	
}

void main(void)
{
	struct device *dev;

	dev = device_get_binding(CONFIG_MPU6050_NAME);
	if (!dev) {
		printk("Failed to get device binding");
		return;
	}

	printk("device is %p, name is %s\n", dev, dev->config->name);

	do_main(dev);
}
