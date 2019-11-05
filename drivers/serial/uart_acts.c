/*
 * Copyright (c) 2019 Actions (Zhuhai) Technology Co., Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief Driver for Actions SoC Uart
 */

#include <kernel.h>
#include <board.h>
#include <init.h>
#include <uart.h>

/* UART registers struct */
struct acts_uart_controller {
	/* UART registers */
	volatile u32_t	ctrl;
	volatile u32_t	rxdat;
	volatile u32_t	txdat;
	volatile u32_t	stat;
	volatile u32_t	br;
};

/* Device data structure */
struct uart_acts_dev_data_t {
	u32_t baud_rate;	        /**< Baud rate */
	u32_t clock_id;
	u32_t reset_id;

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
u32_t ctl_reg_val;
u32_t baud_reg_val;
#endif

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t     cb;     /**< Callback function pointer */
	void *cb_data;	/* Interrupt Callback Arg */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

/* convenience defines */
#define DEV_CFG(dev) \
	((const struct uart_device_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct uart_acts_dev_data_t * const)(dev)->driver_data)
#define UART_STRUCT(dev) \
	((volatile struct acts_uart_controller *)(DEV_CFG(dev))->base)

static const struct uart_driver_api uart_acts_driver_api;

#define UART_CTL_RXENABLE       31
#define UART_CTL_TXENABLE       30
#define UART_CTL_DBGSEL_E       25
#define UART_CTL_DBGSEL_SHIFT   24
#define UART_CTL_DBGSEL_MASK    (0X3<<24)
#define UART_CTL_TX_FIFO_EN     23
#define UART_CTL_RX_FIFO_EN     22
#define UART_CTL_LBEN           20
#define UART_CTL_TXIE           19
#define UART_CTL_RXIE           18
#define UART_CTL_TXDE           17
#define UART_CTL_RXDE           16
#define UART_CTL_EN             15
#define UART_CTL_RTSE           13
#define UART_CTL_AFE            12
#define UART_CTL_RDIC_E         11
#define UART_CTL_RDIC_SHIFT     10
#define UART_CTL_RDIC_MASK      (0X3<<10)
#define UART_CTL_TDIC_E         9
#define UART_CTL_TDIC_SHIFT     8
#define UART_CTL_TDIC_MASK      (0X3<<8)
#define UART_CTL_CTSE           7
#define UART_CTL_PRS_E          6
#define UART_CTL_PRS_SHIFT      4
#define UART_CTL_PRS_MASK       (0X7<<4)
#define UART_CTL_WUEN           3
#define UART_CTL_STPS           2
#define UART_CTL_DWLS_E         1
#define UART_CTL_DWLS_SHIFT     0
#define UART_CTL_DWLS_MASK      (0X3<<0)

#define UART_STA_WSTA           24
#define UART_STA_PAER           23
#define UART_STA_STER           22
#define UART_STA_UTBB           21
#define UART_STA_TXFL_E         20
#define UART_STA_TXFL_SHIFT     16
#define UART_STA_TXFL_MASK      (0X1F<<16)
#define UART_STA_RXFL_E         15
#define UART_STA_RXFL_SHIFT     11
#define UART_STA_RXFL_MASK      (0X1F<<11)
#define UART_STA_TFES           10
#define UART_STA_RFFS           9
#define UART_STA_RTSS           8
#define UART_STA_CTSS           7
#define UART_STA_TFFU           6
#define UART_STA_RFEM           5
#define UART_STA_RXST           4
#define UART_STA_TFER           3
#define UART_STA_RXER           2
#define UART_STA_TIP            1
#define UART_STA_RIP            0

#define UART_BR_TXBRDIV_E       27
#define UART_BR_TXBRDIV_SHIFT   16
#define UART_BR_TXBRDIV_MASK    (0XFFF<<16)
#define UART_BR_RXBRDIV_E       11
#define UART_BR_RXBRDIV_SHIFT   0
#define UART_BR_RXBRDIV_MASK    (0XFFF<<0)

/**
 * @brief Set the baud rate
 *
 * This routine set the given baud rate for the UART.
 *
 * @param dev UART device struct
 * @param baudrate Baud rate
 * @param sys_clk_freq_hz System clock frequency in Hz
 *
 * @return N/A
 */

static int baudrate_set(struct device *dev, u32_t baudrate)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);
	u32_t uart_clock = 16000000;

	uart->br = ((uart_clock / baudrate) << UART_BR_TXBRDIV_SHIFT) |
						((uart_clock / baudrate) << UART_BR_RXBRDIV_SHIFT);

	return 0;
}

/**
 * @brief Initialize UART channel
 *
 * This routine is called to reset the chip in a quiescent state.
 * It is assumed that this function is called only once per UART.
 *
 * @param dev UART device struct
 *
 * @return 0 on success
 */
int uart_acts_init(struct device *dev)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);
	int err;

	/* reset uart */
	acts_reset_peripheral(DEV_DATA(dev)->reset_id);

	/* enable uart clock */
	acts_clock_peripheral_enable(DEV_DATA(dev)->clock_id);

	/* Set baud rate */
	err = baudrate_set(dev, DEV_DATA(dev)->baud_rate);
	if (err)
		return err;

	/* Enable receiver and transmitter */
	uart->ctrl = (0x1 << 31) | (0x1 << 30) |
				(0x1 << 23) | (0x1 << 22) |
				(0x1 << 15) |
				0x3;
	dev->driver_api = &uart_acts_driver_api;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	DEV_CFG(dev)->irq_config_func(dev);
#endif

	return 0;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */

static int uart_acts_poll_in(struct device *dev, unsigned char *c)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	if ((uart->stat & (0x1 << UART_STA_RFEM)) == 0) {
		*c = (unsigned char)uart->rxdat;
		return 0;
	}

	return -1;
}

/**
 * @brief Output a character in polled mode.
 *
 * Checks if the transmitter is empty. If empty, a character is written to
 * the data register.
 *
 * @param dev UART device struct
 * @param c Character to send
 *
 * @return Sent character
 */
static unsigned char uart_acts_poll_out(struct device *dev,
					unsigned char c)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	while ((uart->stat & (1 << UART_STA_TFFU)) != 0)
		;

	uart->txdat = c;

	return c;
}


#ifdef CONFIG_UART_INTERRUPT_DRIVEN

/** Interrupt driven FIFO fill function */
static int uart_acts_fifo_fill(struct device *dev, const u8_t *tx_data, int len)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);
	u8_t num_tx = 0;

	/* Clear the interrupt */
	uart->stat = (1 << UART_STA_TIP);

	while (len - num_tx > 0) {
		while ((uart->stat & (1 << UART_STA_TFFU)) != 0)
			;
		uart->txdat = tx_data[num_tx++];
	}

	return (int)num_tx;
}

/** Interrupt driven FIFO read function */
static int uart_acts_fifo_read(struct device *dev, u8_t *rx_data, const int size)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);
	u8_t num_rx = 0;

	while ((size - num_rx > 0) &&
	       ((uart->stat & (1 << UART_STA_RFEM)) == 0)) {

		rx_data[num_rx++] = (unsigned char)uart->rxdat;
	}

	/* Clear the interrupt */
	uart->stat = (1 << UART_STA_RIP);

	return num_rx;
}

/** Interrupt driven transfer enabling function */
static void uart_acts_irq_tx_enable(struct device *dev)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	uart->ctrl |= (0x1 << UART_CTL_TXIE);
}

/** Interrupt driven transfer disabling function */
static void uart_acts_irq_tx_disable(struct device *dev)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	uart->ctrl &= ~(0x1 << UART_CTL_TXIE);
}

/** Interrupt driven transfer ready function */
static int uart_acts_irq_tx_ready(struct device *dev)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	if (uart->ctrl & (0x1 << UART_CTL_TXIE))
		return (uart->stat & (0x1 << UART_STA_TIP));
	else
		return 0;
}

/** Interrupt driven receiver enabling function */
static void uart_acts_irq_rx_enable(struct device *dev)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	uart->ctrl |= (0x1 << UART_CTL_RXIE);
}

/** Interrupt driven receiver disabling function */
static void uart_acts_irq_rx_disable(struct device *dev)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	uart->ctrl &= ~(0x1 << UART_CTL_RXIE);
}

/** Interrupt driven transfer empty function */
static int uart_acts_irq_tx_complete(struct device *dev)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	return !(uart->stat & (0x1 << UART_STA_UTBB));
}

/** Interrupt driven receiver ready function */
static int uart_acts_irq_rx_ready(struct device *dev)
{
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	if (uart->ctrl & (0x1 << UART_CTL_RXIE))
		return (uart->stat & (0x1 << UART_STA_RIP));
	else
		return 0;
}

/** Interrupt driven pending status function */
static int uart_acts_irq_is_pending(struct device *dev)
{
	return (uart_acts_irq_tx_ready(dev) || uart_acts_irq_rx_ready(dev));
}

/** Interrupt driven interrupt update function */
static int uart_acts_irq_update(struct device *dev)
{
	return 1;
}

/** Set the callback function */
static void uart_acts_irq_callback_set(struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct uart_acts_dev_data_t * const dev_data = DEV_DATA(dev);

	dev_data->cb = cb;
	dev_data->cb_data = cb_data;
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 *
 * @return N/A
 */
void uart_acts_isr(void *arg)
{
	struct device *dev = arg;
	struct uart_acts_dev_data_t * const dev_data = DEV_DATA(dev);

	if (dev_data->cb) {
		dev_data->cb(dev_data->cb_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_driver_api uart_acts_driver_api = {
	.poll_in          = uart_acts_poll_in,          /** Console I/O function */
	.poll_out         = uart_acts_poll_out,         /** Console I/O function */
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill        = uart_acts_fifo_fill,        /** IRQ FIFO fill function */
	.fifo_read        = uart_acts_fifo_read,        /** IRQ FIFO read function */
	.irq_tx_enable    = uart_acts_irq_tx_enable,    /** IRQ transfer enabling function */
	.irq_tx_disable   = uart_acts_irq_tx_disable,   /** IRQ transfer disabling function */
	.irq_tx_ready     = uart_acts_irq_tx_ready,     /** IRQ transfer ready function */
	.irq_rx_enable    = uart_acts_irq_rx_enable,    /** IRQ receiver enabling function */
	.irq_rx_disable   = uart_acts_irq_rx_disable,   /** IRQ receiver disabling function */
	.irq_tx_complete  = uart_acts_irq_tx_complete,  /** IRQ transfer complete function */
	.irq_rx_ready     = uart_acts_irq_rx_ready,     /** IRQ receiver ready function */
	.irq_is_pending   = uart_acts_irq_is_pending,   /** IRQ pending status function */
	.irq_update       = uart_acts_irq_update,       /** IRQ interrupt update function */
	.irq_callback_set = uart_acts_irq_callback_set, /** Set the callback function */
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

#ifdef CONFIG_DEVICE_POWER_MANAGEMENT
static int uart_acts_suspend_device(struct device *dev)
{
	struct uart_acts_dev_data_t * const dev_data = DEV_DATA(dev);
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	dev_data->baud_reg_val = uart->br;
	dev_data->ctl_reg_val = uart->ctrl;

	acts_clock_peripheral_disable(dev_data->clock_id);

	return 0;
}

static int uart_acts_resume_device_from_suspend(struct device *dev)
{
	struct uart_acts_dev_data_t * const dev_data = DEV_DATA(dev);
	volatile struct acts_uart_controller *uart = UART_STRUCT(dev);

	acts_reset_peripheral(dev_data->reset_id);
	acts_clock_peripheral_enable(dev_data->clock_id);

	uart->br = dev_data->baud_reg_val;
	uart->ctrl = dev_data->ctl_reg_val;

	return 0;
}

#endif /* CONFIG_DEVICE_POWER_MANAGEMENT */


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
/* Forward declare function */
static void uart_acts_irq_config_0(struct device *port);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_device_config uart_acts_dev_cfg_0 = {
	.base = (u8_t *)UART0_REG_BASE,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_acts_irq_config_0,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static struct uart_acts_dev_data_t uart_acts_dev_data_0 = {
	.baud_rate = 115200,
	.clock_id = CLOCK_ID_UART0,
	.reset_id = RESET_ID_UART0,
};

DEVICE_INIT(uart_acts_0, "UART_0", &uart_acts_init,
	    &uart_acts_dev_data_0, &uart_acts_dev_cfg_0,
	    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_acts_irq_config_0(struct device *port)
{
	IRQ_CONNECT(IRQ_ID_UART0,
		    0,
		    uart_acts_isr, DEVICE_GET(uart_acts_0),
		    0);
	irq_enable(IRQ_ID_UART0);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
/* Forward declare function */
static void uart_acts_irq_config_1(struct device *port);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_device_config uart_acts_dev_cfg_1 = {
	.base = (u8_t *)UART1_REG_BASE,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_acts_irq_config_1,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static struct uart_acts_dev_data_t uart_acts_dev_data_1 = {
	.baud_rate = 115200,
	.clock_id = CLOCK_ID_UART1,
	.reset_id = RESET_ID_UART1,
};

DEVICE_INIT(uart_acts_1, "UART_1", &uart_acts_init,
	    &uart_acts_dev_data_1, &uart_acts_dev_cfg_1,
	    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_acts_irq_config_1(struct device *port)
{
	IRQ_CONNECT(IRQ_ID_UART1,
		    0,
		    uart_acts_isr, DEVICE_GET(uart_acts_1),
		    0);
	irq_enable(IRQ_ID_UART1);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
/* Forward declare function */
static void uart_acts_irq_config_2(struct device *port);
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static const struct uart_device_config uart_acts_dev_cfg_2 = {
	.base = (u8_t *)UART2_REG_BASE,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = uart_acts_irq_config_2,
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
};

static struct uart_acts_dev_data_t uart_acts_dev_data_2 = {
	.baud_rate = 115200,
	.clock_id = CLOCK_ID_UART2,
	.reset_id = RESET_ID_UART2,
};

DEVICE_INIT(uart_acts_2, "UART_2", &uart_acts_init,
	    &uart_acts_dev_data_2, &uart_acts_dev_cfg_2,
	    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE);


#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void uart_acts_irq_config_2(struct device *port)
{
	IRQ_CONNECT(IRQ_ID_UART2,
		    2,
		    uart_acts_isr, DEVICE_GET(uart_acts_2),
		    0);
	irq_enable(IRQ_ID_UART2);
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */
