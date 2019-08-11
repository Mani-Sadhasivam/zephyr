/*
 * Copyright (c) 2018 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <stdio.h>
#include <string.h>
#include <kernel.h>
#include <device.h>
#include <fsl_sdma.h>
#include <init.h>
#include <drivers/dma.h>
#include <soc.h>

#define MCUX_MAX_CHAN 7

#define LOG_LEVEL CONFIG_DMA_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(dma_mcux);

sdma_handle_t sdma_handle = {0};
AT_NONCACHEABLE_SECTION_ALIGN(sdma_context_data_t context, 4);

extern void SDMA1_DriverIRQHandler(void);

struct mcux_dma_dev_cfg {
	SDMAARM_Type *base;
	void (*irq_config)(void);
	u32_t irq_id;
};

struct mcux_dma_data {
	void *callback_arg;
	void (*dma_callback)(void *arg, u32_t id,
			     int error_code);
};

#define DEV_CFG(dev) \
	((const struct mcux_dma_dev_cfg * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct mcux_dma_data * const)(dev)->driver_data)
#define DEV_BASE(dev) \
	((SDMAARM_Type *)(DEV_CFG(dev))->base)

void mcux_dma_callback(sdma_handle_t *handle, void *param, bool transferDone,
			uint32_t bds)
{
	struct device *dev = (struct device *)param;
	struct mcux_dma_data *data = DEV_DATA(dev);

	/* Simply call user callback routine */
	data->dma_callback(data->callback_arg, 0, 0);
}

static int mcux_dma_config(struct device *dev, u32_t channel,
			 struct dma_config *cfg)
{
	SDMAARM_Type *base = DEV_BASE(dev);
	struct mcux_dma_data *data = DEV_DATA(dev);
	sdma_transfer_config_t transferConfig = {0U};
	u32_t src_bus_width  = dma_width_index(cfg->source_data_size);
	u32_t dst_bus_width  = dma_width_index(cfg->dest_data_size);

	if (channel >= MCUX_MAX_CHAN) {
		return -EINVAL;
	}

	__ASSERT_NO_MSG(cfg->source_data_size == cfg->dest_data_size);
	__ASSERT_NO_MSG(cfg->source_burst_length == cfg->dest_burst_length);

	SDMA_CreateHandle(&sdma_handle, base, channel, &context);

	data->dma_callback = cfg->dma_callback;
	data->callback_arg = cfg->callback_arg;
	SDMA_SetCallback(&sdma_handle, mcux_dma_callback, dev);

	switch (cfg->channel_direction) {
	case MEMORY_TO_MEMORY:
		SDMA_PrepareTransfer(&transferConfig,
				(uint32_t)cfg->head_block->source_address,
				(uint32_t)cfg->head_block->dest_address,
				src_bus_width, dst_bus_width, src_bus_width,
				cfg->head_block->block_size, 0,
				kSDMA_PeripheralTypeMemory,
				kSDMA_MemoryToMemory);
		break;
	default:
		printk("DMA error: Direction not supported: %d",
			    cfg->channel_direction);
		return -EINVAL;
	}
	SDMA_SubmitTransfer(&sdma_handle, &transferConfig);
	SDMA_SetChannelPriority(base, channel, 2U);

	return 0;
}

static int mcux_dma_transfer_start(struct device *dev, u32_t channel)
{
	/* Start transfer */
	SDMA_StartTransfer(&sdma_handle);

	return 0;
}

static int mcux_dma_transfer_stop(struct device *dev, u32_t channel)
{
	return 0;
}

static int mcux_dma_reload(struct device *dev, u32_t id,
			    u32_t src, u32_t dst, size_t size)
{
	return 0;
}

static void mcux_dma_isr(void *arg)
{
	SDMA1_DriverIRQHandler();
}

static int mcux_dma_initialize(struct device *dev)
{
	SDMAARM_Type *base = DEV_BASE(dev);
	const struct mcux_dma_dev_cfg *config = DEV_CFG(dev);
	sdma_config_t userConfig;

	SDMA_GetDefaultConfig(&userConfig);
	SDMA_Init(base, &userConfig);

	/* Configure interrupts */
	config->irq_config();

	/* Enable module's IRQ */
	irq_enable(config->irq_id);

	printk("MCUX DMA ininitalised\n");

	return 0;
}

static const struct dma_driver_api mcux_dma_driver_api = {
	.reload	= mcux_dma_reload,
	.config = mcux_dma_config,
	.start = mcux_dma_transfer_start,
	.stop = mcux_dma_transfer_stop,
};

/* DMA0 */
static struct device DEVICE_NAME_GET(mcux_dma0);
static struct mcux_dma_data mcux_dma_data0;

static void mcux_dma0_irq_config(void)
{
	IRQ_CONNECT(DT_DMA_MCUX_0_IRQ, DT_DMA_MCUX_0_IRQ_PRI,
		    mcux_dma_isr, DEVICE_GET(mcux_dma0), 0);
}

static const struct mcux_dma_dev_cfg mcux_dma0_config = {
	.base = (SDMAARM_Type *)DT_DMA_MCUX_0_BASE_ADDRESS,
	.irq_config = mcux_dma0_irq_config,
	.irq_id = DT_DMA_MCUX_0_IRQ,
};

DEVICE_AND_API_INIT(mcux_dma0, CONFIG_DMA_0_NAME, &mcux_dma_initialize,
		    &mcux_dma_data0, &mcux_dma0_config, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &mcux_dma_driver_api);
