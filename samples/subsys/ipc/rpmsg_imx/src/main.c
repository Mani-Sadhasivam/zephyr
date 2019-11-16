/*
 * Copyright (c) 2019, Manivannan Sadhasivam
 * 
 * Based on NXP SDK 2.6.0
 * Copyright (c) 2018 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/ipm.h>

#include "rpmsg_config.h"
#include "rpmsg_lite.h"
#include "rpmsg_queue.h"
#include "rpmsg_ns.h"

#define RPMsg_LITE_LINK_ID (RL_PLATFORM_IMX8MM_M4_USER_LINK_ID)
#define RPMsg_LITE_SHMEM_BASE 0xB8000000
#define RPMsg_LITE_NS_ANNOUNCE_STRING "rpmsg-virtual-tty-channel-1"
#define RPMsg_LITE_MASTER_IS_LINUX

#define LOCAL_EPT_ADDR (30)
#define BUFFER_SIZE 256

static char app_buf[BUFFER_SIZE];

void app_nameservice_isr_cb(unsigned int new_ept, const char *new_ept_name, unsigned long flags, void *user_data)
{
}

void main()
{
	volatile unsigned long remote_addr;
	struct rpmsg_lite_endpoint *volatile my_ept;
	struct rpmsg_lite_instance *volatile my_rpmsg;
	struct k_msgq *rpmsg_queue;
	int result;

	/* Init RPMSP Lite */
	my_rpmsg = rpmsg_lite_remote_init((void *)RPMsg_LITE_SHMEM_BASE,
					  RPMsg_LITE_LINK_ID, RL_NO_FLAGS);

	/* Wait for the master to establish link */
	while (!rpmsg_lite_is_link_up(my_rpmsg));
	printk("Link is up!\r\n");

	/* Create RPMsg queue */
	rpmsg_queue = rpmsg_queue_create(my_rpmsg);

	/* Create RPMsg endpoint */
	my_ept = rpmsg_lite_create_ept(my_rpmsg, LOCAL_EPT_ADDR,
				       rpmsg_queue_rx_cb,
				       (void *)rpmsg_queue);

	/* Bind the channel and announce nameservice to master */
	rpmsg_ns_bind(my_rpmsg, app_nameservice_isr_cb, NULL);
	rpmsg_ns_announce(my_rpmsg, my_ept, RPMsg_LITE_NS_ANNOUNCE_STRING,
			  RL_NS_CREATE);

	printk("Nameservice announce sent.\r\n");

	/*
	 * BLOCKING: Wait for the greeting message from master. This comes
	 * during the probe of imx_rpmsg_tty module.
	 */
	result = rpmsg_queue_recv(my_rpmsg, rpmsg_queue,
				  (unsigned long *)&remote_addr, app_buf,
				  sizeof(app_buf), NULL, RL_BLOCK);

	if (result != 0) {
		printk("RPMsg receive failed: %d\n", result);
		return;
	}

	printk("Successfully established connection with master!\n");

	while(1) {
		memset(app_buf, 0, BUFFER_SIZE *sizeof(app_buf[0]));

		/* BLOCKING: Get RPMsg buffer from master */
		result = rpmsg_queue_recv(my_rpmsg, rpmsg_queue,
					  (unsigned long *)&remote_addr,
					  app_buf, sizeof(app_buf), NULL,
					  RL_BLOCK);
		if (result != 0) {
			printk("RPMsg receive failed: %d\n", result);
			return;
		}

		/*
		 * imx_rpmsg_tty module sends a newline after each message.
		 * We should just ignore.
		 */
		if ((app_buf[0] == 0xd) && (app_buf[1] == 0xa))
			continue;

		printk("Got Message From Master: %s\r\n", app_buf);
	}
}
