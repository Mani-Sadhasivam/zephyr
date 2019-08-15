/*
 * Copyright (c) 2017 comsuisse AG
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/clock_control.h>
#include <drivers/i2s.h>
#include <fsl_clock.h>
#include <fsl_sai.h>
#include <fsl_sai_sdma.h>
#include <soc.h>

#define SAI_TX_SOURCE 5

#define LOG_LEVEL CONFIG_I2S_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(i2s_mcux);

sai_handle_t txHandle = {0};
sai_sdma_handle_t SdmatxHandle = {0};
sdma_handle_t dmaHandle = {0};
sdma_context_data_t context = {0};

extern void SDMA2_DriverIRQHandler(void);

/* Device constant configuration parameters */
struct i2s_mcux_config {
	I2S_Type *base;
	char *clock_name;
	clock_control_subsys_t clock_subsys;
	void (*irq_config)(struct device *dev);
	u32_t irq_id;
};

/* Device run time data */
struct i2s_mcux_data {
	struct device *clock_dev;
	status_t callback_status;
	sai_handle_t tx_handle;
	sai_transfer_t xfer;
	sai_master_clock_t mclkConfig;
	sai_transceiver_t sai_config;
};

#define DEV_CFG(dev) \
	((const struct i2s_mcux_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2s_mcux_data * const)(dev)->driver_data)
#define DEV_BASE(dev) \
	((I2S_Type *)(DEV_CFG(dev))->base)

static int i2s_mcux_configure(struct device *dev, enum i2s_dir dir,
			     struct i2s_config *i2s_cfg)
{
	I2S_Type *base = DEV_BASE(dev);
	const struct i2s_mcux_config *config = DEV_CFG(dev);
	struct i2s_mcux_data *data = DEV_DATA(dev);
	u32_t clock_freq;
	sai_word_width_t word_width;
	sai_sample_rate_t sample_rate;

	if (clock_control_get_rate(data->clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}

	switch(i2s_cfg->word_size) {
	case 8:
		word_width = kSAI_WordWidth8bits;
		break;
	case 16:
		word_width = kSAI_WordWidth16bits;
		break;
	case 24:
		word_width = kSAI_WordWidth24bits;
		break;
	case 32:
		word_width = kSAI_WordWidth32bits;
		break;
	default:
		printk("Unsupported word size");
		return -EINVAL;
	}

	switch(i2s_cfg->frame_clk_freq) {
	case 8000:
		sample_rate = kSAI_SampleRate8KHz;
		break;
	case 16000:
		sample_rate = kSAI_SampleRate16KHz;
		break;
	default:
		printk("Unsupported sample rate");
		return -EINVAL;
	}

	SAI_GetClassicI2SConfig(&data->sai_config, word_width,
				kSAI_Stereo, kSAI_Channel0Mask);

	data->sai_config.bitClock.bclkSource = (sai_bclk_source_t)1U;
	SAI_TransferTxSetConfigSDMA(base, &SdmatxHandle, &data->sai_config);

	/* set bit clock divider */
	SAI_TxSetBitClockRate(base, clock_freq, sample_rate, word_width,
				i2s_cfg->channels);

	/* master clock configurations */
	data->mclkConfig.mclkOutputEnable = true;
	SAI_SetMasterClockConfig(base, &data->mclkConfig);

//	SAI_TxEnableInterrupts(base, kSAI_FIFOErrorInterruptEnable);

	return 0;
}

static int i2s_mcux_trigger(struct device *dev, enum i2s_dir dir,
			   enum i2s_trigger_cmd cmd)
{
	I2S_Type *base = DEV_BASE(dev);
	struct i2s_mcux_data *data = DEV_DATA(dev);

	switch (cmd) {
	case I2S_TRIGGER_START:
		SAI_TransferSendSDMA(base, &SdmatxHandle, &data->xfer);
		break;

	default:
		printk("Unsupported trigger command");
		return -EINVAL;
	}

	return 0;
}

static int i2s_mcux_write(struct device *dev, void *mem_block, size_t size)
{
	struct i2s_mcux_data *data = DEV_DATA(dev);

	data->xfer.data	= (u8_t *)mem_block;
	data->xfer.dataSize = size;

	return 0;
}

static void i2s_mcux_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	I2S_Type *base = DEV_BASE(dev);

	SAI_TransferTxHandleIRQ(base, &txHandle);
}

static void sai_dma_isr(void *arg)
{
	SDMA2_DriverIRQHandler();
}

static void sai_sdma_callback(I2S_Type *base, sai_sdma_handle_t *handle, status_t status,
			void *userData)
{
	struct device *dev = userData;
	struct i2s_mcux_data *data = DEV_DATA(dev);

	ARG_UNUSED(handle);
	ARG_UNUSED(base);

	data->callback_status = status;
}

static int i2s_mcux_init(struct device *dev)
{
	I2S_Type *base = DEV_BASE(dev);
	const struct i2s_mcux_config *config = DEV_CFG(dev);
	struct i2s_mcux_data *data = DEV_DATA(dev);
	struct device *clock_dev;

	clock_dev = device_get_binding(config->clock_name);
	if (clock_dev == NULL) {
		return -EINVAL;
	}

	data->clock_dev = clock_dev;

	sdma_config_t dmaConfig = {0};
	/* Create SDMA handle */
	SDMA_GetDefaultConfig(&dmaConfig);
	dmaConfig.ratio = kSDMA_ARMClockFreq;
	SDMA_Init(SDMAARM2, &dmaConfig);
	SDMA_CreateHandle(&dmaHandle, SDMAARM2, 1, &context);
	SDMA_SetChannelPriority(SDMAARM2, 1, 2);

	SAI_TransferTxCreateHandleSDMA(base, &SdmatxHandle, sai_sdma_callback,
					NULL, &dmaHandle, SAI_TX_SOURCE);
	/* Configure interrupts */
	config->irq_config(dev);

	printk("I2S MCUX initialized\n");

	return 0;
}

static const struct i2s_driver_api i2s_mcux_driver_api = {
	.configure = i2s_mcux_configure,
	.write = i2s_mcux_write,
	.trigger = i2s_mcux_trigger,
};

#ifdef CONFIG_I2S_3
static void i2s_mcux_config_func_3(struct device *dev);

static const struct i2s_mcux_config i2s_mcux_config_3 = {
	.base = (I2S_Type *)DT_I2S_MCUX_3_BASE_ADDRESS,
	.clock_name = DT_I2S_MCUX_3_CLOCK_NAME,
	.clock_subsys = (clock_control_subsys_t)DT_I2S_MCUX_3_CLOCK_SUBSYS,
	.irq_config = i2s_mcux_config_func_3,
};

static struct i2s_mcux_data i2s_mcux_data_3;

DEVICE_AND_API_INIT(i2s_mcux_3, CONFIG_I2S_3_NAME, &i2s_mcux_init,
		    &i2s_mcux_data_3, &i2s_mcux_config_3,
		    POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &i2s_mcux_driver_api);

static void i2s_mcux_config_func_3(struct device *dev)
{
	IRQ_CONNECT(DT_I2S_MCUX_3_IRQ, DT_I2S_MCUX_3_IRQ_PRI,
		    i2s_mcux_isr, DEVICE_GET(i2s_mcux_3), 0);

	irq_enable(DT_I2S_MCUX_3_IRQ);

	IRQ_CONNECT(CONFIG_SAI_DMA_IRQ, CONFIG_SAI_DMA_IRQ_PRI,
		    sai_dma_isr, DEVICE_GET(i2s_mcux_3), 0);

	irq_enable(CONFIG_SAI_DMA_IRQ);
}

#endif /* CONFIG_I2S_3 */
