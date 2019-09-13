/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>

void main(void)
{
	struct device *dev = device_get_binding("BME280");

	if (dev == NULL) {
		printk("Could not get BME280 device\n");
		return;
	}

	printk("dev %p name %s\n", dev, dev->config->name);

	while (1) {
		struct sensor_value temp;

		sensor_sample_fetch(dev);
		sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

		printk("temp: %d.%06d\n", temp.val1, temp.val2);

		k_sleep(1000);
	}
}
