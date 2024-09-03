/*
 * SPI serial driver for USB to Quad UARTs chip ch9434.
 *
 * Copyright (C) 2024 Nanjing Qinheng Microelectronics Co., Ltd.
 * Web:      http://wch.cn
 * Author:   WCH <tech@wch.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Update Log:
 * V1.0 - initial version
 * V1.1 - change RS485 setting and getting method, add procfs for dump registers
 *      - add support for CH9434A with continuous spi transmission
 * V1.2 - add support for kernel version beyond 5.12.18
 * V1.3 - improve spi transfer compatibility on some platforms
 */

#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include "linux/version.h"
#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/serial_reg.h>
#include <linux/spi/spi.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>

#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC   "SPI serial driver for ch9434."
#define VERSION_DESC  "V1.3 On 2024.09"

#ifndef PORT_SC16IS7XX
#define PORT_SC16IS7XX 128
#endif

#define USE_IRQ_FROM_DTS
#define GPIO_NUMBER 0
#define USE_SPI_MODE
#define CH943X_NAME_SPI "ch943x_spi"

#define IOCTL_MAGIC	       'W'
#define IOCTL_CMD_GPIOENABLE   _IOW(IOCTL_MAGIC, 0x80, u16)
#define IOCTL_CMD_GPIODIR      _IOW(IOCTL_MAGIC, 0x81, u16)
#define IOCTL_CMD_GPIOPULLUP   _IOW(IOCTL_MAGIC, 0x82, u16)
#define IOCTL_CMD_GPIOPULLDOWN _IOW(IOCTL_MAGIC, 0x83, u16)
#define IOCTL_CMD_GPIOSET      _IOW(IOCTL_MAGIC, 0x84, u16)
#define IOCTL_CMD_GPIOGET      _IOWR(IOCTL_MAGIC, 0x85, u16)
#define IOCTL_CMD_GRS485       _IOR(IOCTL_MAGIC, 0x86, u16)
#define IOCTL_CMD_SRS485       _IOW(IOCTL_MAGIC, 0x87, u16)

/* CH943X register definitions */
#define CH943X_RHR_REG (0x00) /* RX FIFO */
#define CH943X_THR_REG (0x00) /* TX FIFO */
#define CH943X_IER_REG (0x01) /* Interrupt enable */
#define CH943X_IIR_REG (0x02) /* Interrupt Identification */
#define CH943X_FCR_REG (0x02) /* FIFO control */
#define CH943X_LCR_REG (0x03) /* Line Control */
#define CH943X_MCR_REG (0x04) /* Modem Control */
#define CH943X_LSR_REG (0x05) /* Line Status */
#define CH943X_MSR_REG (0x06) /* Modem Status */
#define CH943X_SPR_REG (0x07) /* Scratch Pad */

#define CH943X_CLK_REG	  (0x48) /* Clock Set */
#define CH943X_RS485_REG  (0x41) /* RS485 Control */
#define CH943X_FIFO_REG	  (0x42) /* FIFO Control */
#define CH943X_FIFOCL_REG (0x43) /* FIFO Count Low */
#define CH943X_FIFOCH_REG (0x44) /* FIFO Count High */

#define CH943X_GPIOEN_REG  (0x50) /* GPIO Enable Set */
#define CH943X_GPIODIR_REG (0x54) /* GPIO Direction Set */
#define CH943X_GPIOPU_REG  (0x58) /* GPIO PullUp Set */
#define CH943X_GPIOPD_REG  (0x5C) /* GPIO PullDown Set */
#define CH943X_GPIOVAL_REG (0x60) /* GPIO Value Set */

#define CH943X_SPI_CONT_MODE_REG (0x64) /* SPI transfer mode Set */
#define CH943X_CHIP_VER_REG	 (0x65) /* Firmware Version */

/* Special Register set: Only if (LCR[7] == 1) */
#define CH943X_DLL_REG (0x00) /* Divisor Latch Low */
#define CH943X_DLH_REG (0x01) /* Divisor Latch High */

/* IER register bits */
#define CH943X_IER_RDI_BIT  (1 << 0) /* Enable RX data interrupt */
#define CH943X_IER_THRI_BIT (1 << 1) /* Enable TX holding register interrupt */
#define CH943X_IER_RLSI_BIT (1 << 2) /* Enable RX line status interrupt */
#define CH943X_IER_MSI_BIT  (1 << 3) /* Enable Modem status interrupt */

/* IER enhanced register bits */
#define CH943X_IER_RESET_BIT	(1 << 7) /* Enable Soft reset */
#define CH943X_IER_LOWPOWER_BIT (1 << 6) /* Enable low power mode */
#define CH943X_IER_SLEEP_BIT	(1 << 5) /* Enable sleep mode */

/* FCR register bits */
#define CH943X_FCR_FIFO_BIT    (1 << 0) /* Enable FIFO */
#define CH943X_FCR_RXRESET_BIT (1 << 1) /* Reset RX FIFO */
#define CH943X_FCR_TXRESET_BIT (1 << 2) /* Reset TX FIFO */
#define CH943X_FCR_RXLVLL_BIT  (1 << 6) /* RX Trigger level LSB */
#define CH943X_FCR_RXLVLH_BIT  (1 << 7) /* RX Trigger level MSB */

/* IIR register bits */
#define CH943X_IIR_NO_INT_BIT (1 << 0) /* No interrupts pending */
#define CH943X_IIR_ID_MASK    0x0e     /* Mask for the interrupt ID */
#define CH943X_IIR_THRI_SRC   0x02     /* TX holding register empty */
#define CH943X_IIR_RDI_SRC    0x04     /* RX data interrupt */
#define CH943X_IIR_RLSE_SRC   0x06     /* RX line status error */
#define CH943X_IIR_RTOI_SRC   0x0c     /* RX time-out interrupt */
#define CH943X_IIR_MSI_SRC    0x00     /* Modem status interrupt */

/* LCR register bits */
#define CH943X_LCR_LENGTH0_BIT (1 << 0) /* Word length bit 0 */
#define CH943X_LCR_LENGTH1_BIT \
	(1 << 1) /* Word length bit 1       \
              *                         \
              * Word length bits table: \
              * 00 -> 5 bit words       \
              * 01 -> 6 bit words       \
              * 10 -> 7 bit words       \
              * 11 -> 8 bit words       \
              */
#define CH943X_LCR_STOPLEN_BIT \
	(1 << 2)			    /* STOP length bit            \
                                             *                            \
                                             * STOP length bit table:     \
                                             * 0 -> 1 stop bit            \
                                             * 1 -> 1-1.5 stop bits if    \
                                             *      word length is 5,     \
                                             *      2 stop bits otherwise \
                                             */
#define CH943X_LCR_PARITY_BIT	   (1 << 3) /* Parity bit enable */
#define CH943X_LCR_ODDPARITY_BIT   (0)	    /* Odd parity bit enable */
#define CH943X_LCR_EVENPARITY_BIT  (1 << 4) /* Even parity bit enable */
#define CH943X_LCR_MARKPARITY_BIT  (1 << 5) /* Mark parity bit enable */
#define CH943X_LCR_SPACEPARITY_BIT (3 << 4) /* Space parity bit enable */

#define CH943X_LCR_TXBREAK_BIT (1 << 6) /* TX break enable */
#define CH943X_LCR_DLAB_BIT    (1 << 7) /* Divisor Latch enable */
#define CH943X_LCR_WORD_LEN_5  (0x00)
#define CH943X_LCR_WORD_LEN_6  (0x01)
#define CH943X_LCR_WORD_LEN_7  (0x02)
#define CH943X_LCR_WORD_LEN_8  (0x03)
#define CH943X_LCR_CONF_MODE_A CH943X_LCR_DLAB_BIT /* Special reg set */

/* MCR register bits */
#define CH943X_MCR_DTR_BIT  (1 << 0) /* DTR complement */
#define CH943X_MCR_RTS_BIT  (1 << 1) /* RTS complement */
#define CH943X_MCR_OUT1	    (1 << 2) /* OUT1 */
#define CH943X_MCR_OUT2	    (1 << 3) /* OUT2 */
#define CH943X_MCR_LOOP_BIT (1 << 4) /* Enable loopback test mode */
#define CH943X_MCR_AFE	    (1 << 5) /* Enable Hardware Flow control */

/* LSR register bits */
#define CH943X_LSR_DR_BIT	  (1 << 0) /* Receiver data ready */
#define CH943X_LSR_OE_BIT	  (1 << 1) /* Overrun Error */
#define CH943X_LSR_PE_BIT	  (1 << 2) /* Parity Error */
#define CH943X_LSR_FE_BIT	  (1 << 3) /* Frame Error */
#define CH943X_LSR_BI_BIT	  (1 << 4) /* Break Interrupt */
#define CH943X_LSR_BRK_ERROR_MASK 0x1E	   /* BI, FE, PE, OE bits */
#define CH943X_LSR_THRE_BIT	  (1 << 5) /* TX holding register empty */
#define CH943X_LSR_TEMT_BIT	  (1 << 6) /* Transmitter empty */
#define CH943X_LSR_FIFOE_BIT	  (1 << 7) /* Fifo Error */

/* MSR register bits */
#define CH943X_MSR_DCTS_BIT   (1 << 0) /* Delta CTS Clear To Send */
#define CH943X_MSR_DDSR_BIT   (1 << 1) /* Delta DSR Data Set Ready */
#define CH943X_MSR_DRI_BIT    (1 << 2) /* Delta RI Ring Indicator */
#define CH943X_MSR_DCD_BIT    (1 << 3) /* Delta CD Carrier Detect */
#define CH943X_MSR_CTS_BIT    (1 << 4) /* CTS */
#define CH943X_MSR_DSR_BIT    (1 << 5) /* DSR */
#define CH943X_MSR_RI_BIT     (1 << 6) /* RI */
#define CH943X_MSR_CD_BIT     (1 << 7) /* CD */
#define CH943X_MSR_DELTA_MASK 0x0F     /* Any of the delta bits! */

/* Clock Set */
#define CH943X_CLK_PLL_BIT (1 << 7) /* PLL Enable */
#define CH943X_CLK_EXT_BIT (1 << 6) /* Extenal Clock Enable */

/* FIFO */
#define CH943X_FIFO_RD_BIT (0 << 4) /* Receive FIFO */
#define CH943X_FIFO_WR_BIT (1 << 4) /* Receive FIFO */

/* SPI Cont Mode Set */
#define CH943X_SPI_CONTE_BIT (1 << 0) /* SPI Cont Enable */

/* Misc definitions */
#define CH943X_FIFO_SIZE (1536)
#define CH943X_CMD_DELAY 3

struct ch943x_devtype {
	char name[10];
	int nr_uart;
};

struct ch943x_one {
	struct uart_port port;
	struct work_struct tx_work;
	struct work_struct md_work;
	struct work_struct stop_rx_work;
	struct work_struct stop_tx_work;
	struct serial_rs485 rs485;
	unsigned char msr_reg;
	unsigned char ier;
	unsigned char mcr_force;
	bool isopen;
	unsigned char *txbuf;
	unsigned char *rxbuf;
};

struct ch943x_port {
	struct uart_driver uart;
	struct ch943x_devtype *devtype;
	struct mutex mutex;
	struct mutex mutex_bus_access;
	struct clk *clk;
	struct spi_device *spi_dev;
	u8 reg485;
	bool spi_contmode;
	u8 ver[4];
	unsigned char buf[65536];
	struct ch943x_one p[0];
};

struct ch943x_port *g_ch943x_port;

#define to_ch943x_one(p, e) ((container_of((p), struct ch943x_one, e)))

#ifdef USE_SPI_MODE
static u8 ch943x_port_read(struct uart_port *port, u8 reg)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 cmd = (0x00 | reg) + (port->line * 0x10);
	u8 val;
	ssize_t status;
	struct spi_message m;
	struct spi_transfer t[2] = {
		{
			.tx_buf = &cmd,
			.len = 1,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
		{
			.rx_buf = &val,
			.len = 1,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
	};

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(&s->spi_dev->dev, "Failed to %s Err_code %ld\n", __func__, (unsigned long)status);
	}
	dev_vdbg(&s->spi_dev->dev, "%s - reg:0x%2x, val:0x%2x\n", __func__, cmd, val);

	return val;
}

static u8 ch943x_port_read_specify(struct uart_port *port, u8 portnum, u8 reg)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 cmd = (0x00 | reg) + (portnum * 0x10);
	u8 val;
	ssize_t status;
	struct spi_message m;
	struct spi_transfer t[2] = {
		{
			.tx_buf = &cmd,
			.len = 1,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
		{
			.rx_buf = &val,
			.len = 1,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
	};

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(&s->spi_dev->dev, "Failed to %s Err_code %ld\n", __func__, (unsigned long)status);
	}
	dev_vdbg(&s->spi_dev->dev, "%s - reg:0x%2x, val:0x%2x\n", __func__, cmd, val);

	return val;
}

void ch943x_port_write(struct uart_port *port, u8 reg, u8 val)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 cmd = (0x80 | reg) + (port->line * 0x10);
	ssize_t status;
	struct spi_message m;
	u8 txbuf[2] = { cmd, val };

	struct spi_transfer x = {
		.tx_buf = txbuf,
		.len = 2,
	};

	struct spi_transfer t[2] = {
		{
			.tx_buf = &cmd,
			.len = 1,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
		{
			.tx_buf = &val,
			.len = 1,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
	};

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	if (s->spi_contmode) {
		spi_message_add_tail(&x, &m);
	} else {
		spi_message_add_tail(&t[0], &m);
		spi_message_add_tail(&t[1], &m);
	}
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(&s->spi_dev->dev, "Failed to %s Err_code %ld\n", __func__, (unsigned long)status);
	}
	dev_vdbg(&s->spi_dev->dev, "%s - reg:0x%2x, val:0x%2x\n", __func__, cmd, val);
}

static void ch943x_port_write_spefify(struct uart_port *port, u8 portnum, u8 reg, u8 val)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 cmd = (0x80 | reg) + (portnum * 0x10);
	ssize_t status;
	struct spi_message m;
	u8 txbuf[2] = { cmd, val };

	struct spi_transfer x = {
		.tx_buf = txbuf,
		.len = 2,
	};

	struct spi_transfer t[2] = {
		{
			.tx_buf = &cmd,
			.len = 1,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
		{
			.tx_buf = &val,
			.len = 1,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
	};

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	if (s->spi_contmode) {
		spi_message_add_tail(&x, &m);
	} else {
		spi_message_add_tail(&t[0], &m);
		spi_message_add_tail(&t[1], &m);
	}
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(&s->spi_dev->dev, "Failed to %s Err_code %ld\n", __func__, (unsigned long)status);
	}
	dev_vdbg(&s->spi_dev->dev, "%s - reg:0x%2x, val:0x%2x\n", __func__, cmd, val);
}

static int ch943x_port_read_multi(struct uart_port *port, u8 portnum, u8 reg, u8 *buf, u8 count)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 cmd = (0x00 | reg) + (portnum * 0x10);
	ssize_t status;
	struct spi_message m;
	struct spi_transfer t[2] = {
		{
			.tx_buf = &cmd,
			.len = 1,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
		{
			.rx_buf = buf,
			.len = 4,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
	};

	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&t[0], &m);
	spi_message_add_tail(&t[1], &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(&s->spi_dev->dev, "Failed to %s Err_code %ld\n", __func__, (unsigned long)status);
		goto out;
	}

out:
	return status < 0 ? status : 0;
}

static void ch943x_port_update(struct uart_port *port, u8 reg, u8 mask, u8 val)
{
	unsigned int tmp;

	tmp = ch943x_port_read(port, reg);
	tmp &= ~mask;
	tmp |= val & mask;
	ch943x_port_write(port, reg, tmp);
}

void ch943x_raw_write(struct uart_port *port, const void *reg, unsigned char *buf, int len)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct ch943x_one *one = to_ch943x_one(port, port);
	ssize_t status;
	struct spi_message m;
	int writesum = 0;
	int i;

	struct spi_transfer t[2] = {
		{
			.tx_buf = reg,
			.len = 1,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			.delay.value = CH943X_CMD_DELAY,
#else
			.delay_usecs = CH943X_CMD_DELAY,
#endif
		},
	};

	struct spi_transfer x = {
		.tx_buf = one->txbuf,
		.len = len + 1,
	};

	if (s->spi_contmode) {
		one->txbuf[0] = *(u8 *)reg;
		memcpy(one->txbuf + 1, buf, len);
		mutex_lock(&s->mutex_bus_access);
		spi_message_init(&m);
		spi_message_add_tail(&x, &m);
		status = spi_sync(s->spi_dev, &m);
		mutex_unlock(&s->mutex_bus_access);
		if (status < 0) {
			dev_err(&s->spi_dev->dev, "Failed to %s Err_code %ld\n", __func__, (unsigned long)status);
			return;
		}
		dev_vdbg(&s->spi_dev->dev, "%s - reg:0x%2x, len:%d\n", __func__, *(u8 *)reg, len);
		for (i = 0; i < len; i++)
			dev_vdbg(&s->spi_dev->dev, "\tbuf[%d]:0x%2x\n", i, buf[i]);
	} else {
		while (len--) {
			t[1].tx_buf = buf + writesum;
			t[1].len = 1;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 12, 19))
			t[1].delay.value = CH943X_CMD_DELAY,
#else
			t[1].delay_usecs = CH943X_CMD_DELAY,
#endif
			mutex_lock(&s->mutex_bus_access);
			spi_message_init(&m);
			spi_message_add_tail(&t[0], &m);
			spi_message_add_tail(&t[1], &m);
			status = spi_sync(s->spi_dev, &m);
			mutex_unlock(&s->mutex_bus_access);
			if (status < 0) {
				dev_err(&s->spi_dev->dev, "Failed to %s Err_code %ld\n", __func__,
					(unsigned long)status);
				return;
			}
			dev_vdbg(&s->spi_dev->dev, "%s - reg:0x%2x, buf[%d]:0x%2x\n", __func__, *(u8 *)reg, writesum,
				 buf[writesum]);
			writesum++;
		}
	}
}

static void ch943x_raw_read(struct uart_port *port, u8 reg, unsigned char *buf, int len)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct ch943x_one *one = to_ch943x_one(port, port);
	u8 cmd = (0x00 | reg) + (port->line * 0x10);
	ssize_t status;
	struct spi_message m;

	struct spi_transfer x = {
		.rx_buf = one->rxbuf,
		.tx_buf = one->rxbuf,
		.len = len + 2,
	};

	one->rxbuf[0] = cmd;
	mutex_lock(&s->mutex_bus_access);
	spi_message_init(&m);
	spi_message_add_tail(&x, &m);
	status = spi_sync(s->spi_dev, &m);
	mutex_unlock(&s->mutex_bus_access);
	if (status < 0) {
		dev_err(&s->spi_dev->dev, "Failed to %s Err_code %ld\n", __func__, (unsigned long)status);
	} else {
		memcpy(buf, one->rxbuf + 2, len);
	}
}

#endif

static struct ch943x_devtype ch943x_devtype = {
	.name = "CH943X",
	.nr_uart = 4,
};

static int ch943x_set_baud(struct uart_port *port, int baud)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 lcr;
	unsigned long clk = port->uartclk;
	unsigned long div;

	dev_dbg(&s->spi_dev->dev, "%s - %d\n", __func__, baud);

	div = clk * 10 / 8 / baud;
	div = (div + 5) / 10;

	lcr = ch943x_port_read(port, CH943X_LCR_REG);

	/* Open the LCR divisors for configuration */
	ch943x_port_write(port, CH943X_LCR_REG, CH943X_LCR_CONF_MODE_A);

	/* Write the new divisor */
	ch943x_port_write(port, CH943X_DLH_REG, div / 256);
	ch943x_port_write(port, CH943X_DLL_REG, div % 256);

	/* Put LCR back to the normal mode */
	ch943x_port_write(port, CH943X_LCR_REG, lcr);

	return DIV_ROUND_CLOSEST(clk / 16, div);
}

static int ch943x_dump_register(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 lcr;
	u8 i, reg;

	lcr = ch943x_port_read(port, CH943X_LCR_REG);
	ch943x_port_write(port, CH943X_LCR_REG, CH943X_LCR_CONF_MODE_A);
	dev_vdbg(&s->spi_dev->dev, "******Dump register at LCR=DLAB\n");
	for (i = 0; i < 1; i++) {
		reg = ch943x_port_read(port, i);
		dev_vdbg(&s->spi_dev->dev, "Reg[0x%02x] = 0x%02x\n", i, reg);
	}

	ch943x_port_update(port, CH943X_LCR_REG, CH943X_LCR_CONF_MODE_A, 0);
	dev_vdbg(&s->spi_dev->dev, "******Dump register at LCR=Normal\n");
	for (i = 0; i < 8; i++) {
		reg = ch943x_port_read(port, i);
		dev_vdbg(&s->spi_dev->dev, "Reg[0x%02x] = 0x%02x\n", i, reg);
	}

	/* Put LCR back to the normal mode */
	ch943x_port_write(port, CH943X_LCR_REG, lcr);

	return 0;
}

static int ch943x_scr_test(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	u8 val;

	dev_vdbg(&s->spi_dev->dev, "******Uart %d SPR Test Start******\n", port->line);

	ch943x_port_write(port, CH943X_SPR_REG, 0x55);
	val = ch943x_port_read(port, CH943X_SPR_REG);
	if (val != 0x55) {
		dev_err(&s->spi_dev->dev, "UART %d SPR Test Failed.\n", port->line);
		return -1;
	}

	ch943x_port_write(port, CH943X_SPR_REG, 0xAA);
	val = ch943x_port_read(port, CH943X_SPR_REG);
	if (val != 0xAA) {
		dev_err(&s->spi_dev->dev, "UART %d SPR Test Failed.\n", port->line);
		return -1;
	}

	dev_vdbg(&s->spi_dev->dev, "******Uart %d SPR Test End******\n", port->line);

	return 0;
}

static void ch943x_handle_rx(struct uart_port *port, unsigned int rxlen, unsigned int iir)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	unsigned int lsr = 0, ch, flag, bytes_read = 0, i;
	bool read_lsr = (iir == CH943X_IIR_RLSE_SRC) ? true : false;

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);

	if (unlikely(rxlen >= sizeof(s->buf))) {
		port->icount.buf_overrun++;
		rxlen = sizeof(s->buf);
	}

	/* Only read lsr if there are possible errors in FIFO */
	if (read_lsr) {
		lsr = ch943x_port_read(port, CH943X_LSR_REG);
		/* No errors left in FIFO */
		if (!(lsr & CH943X_LSR_FIFOE_BIT))
			read_lsr = false;
	} else
		lsr = 0;

	if (s->spi_contmode) {
		ch943x_raw_read(port, CH943X_RHR_REG, s->buf, rxlen);
		bytes_read += rxlen;
	} else {
		for (i = 0; i < rxlen; i++) {
			s->buf[i] = ch943x_port_read(port, CH943X_RHR_REG);
			bytes_read++;
		}
	}

	flag = TTY_NORMAL;
	port->icount.rx++;

	if (unlikely(lsr & CH943X_LSR_BRK_ERROR_MASK)) {
		dev_err(&s->spi_dev->dev, "%s - lsr error detect\n", __func__);
		if (lsr & CH943X_LSR_BI_BIT) {
			lsr &= ~(CH943X_LSR_FE_BIT | CH943X_LSR_PE_BIT);
			port->icount.brk++;
			if (uart_handle_break(port))
				goto ignore_char;
		} else if (lsr & CH943X_LSR_PE_BIT)
			port->icount.parity++;
		else if (lsr & CH943X_LSR_FE_BIT)
			port->icount.frame++;
		else if (lsr & CH943X_LSR_OE_BIT)
			port->icount.overrun++;

		lsr &= port->read_status_mask;
		if (lsr & CH943X_LSR_BI_BIT)
			flag = TTY_BREAK;
		else if (lsr & CH943X_LSR_PE_BIT)
			flag = TTY_PARITY;
		else if (lsr & CH943X_LSR_FE_BIT)
			flag = TTY_FRAME;

		if (lsr & CH943X_LSR_OE_BIT)
			dev_err(&s->spi_dev->dev, "%s - overrun detect\n", __func__);
	}

	for (i = 0; i < bytes_read; i++) {
		ch = s->buf[i];

		if (uart_handle_sysrq_char(port, ch))
			continue;

		if (lsr & port->ignore_status_mask)
			continue;

		uart_insert_char(port, lsr, CH943X_LSR_OE_BIT, ch, flag);
	}

	dev_vdbg(&s->spi_dev->dev, "%s - bytes_read:%d\n", __func__, bytes_read);

ignore_char:
	tty_flip_buffer_push(&port->state->port);
}

static void ch943x_handle_tx(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct circ_buf *xmit = &port->state->xmit;
	unsigned int txlen, to_send, i;
	unsigned char thr_reg;

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
	/* xon/xoff char */
	if (unlikely(port->x_char)) {
		ch943x_port_write(port, CH943X_THR_REG, port->x_char);
		port->icount.tx++;
		port->x_char = 0;
		return;
	}

	if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
		dev_vdbg(&s->spi_dev->dev, "ch943x_handle_tx stopped\n");
		ch943x_port_update(port, CH943X_IER_REG, CH943X_IER_THRI_BIT, 0);
		return;
	}

	/* Get length of data pending in circular buffer */
	to_send = uart_circ_chars_pending(xmit);

	if (likely(to_send)) {
		/* Limit to size of TX FIFO */
		txlen = CH943X_FIFO_SIZE;
		to_send = (to_send > txlen) ? txlen : to_send;

		/* Add data to send */
		port->icount.tx += to_send;

		/* Convert to linear buffer */
		for (i = 0; i < to_send; ++i) {
			s->buf[i] = xmit->buf[xmit->tail];
			xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
		}
		dev_vdbg(&s->spi_dev->dev, "ch943x_handle_tx %d bytes\n", to_send);
		thr_reg = (0x80 | CH943X_THR_REG) + (port->line * 0x10);
		ch943x_raw_write(port, &thr_reg, s->buf, to_send);
	}

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);
}

static void ch943x_port_irq(struct ch943x_port *s, int portno)
{
	struct uart_port *port = &s->p[portno].port;
	struct ch943x_one *one = to_ch943x_one(port, port);
	unsigned int iir, msr, rxlen = 0;
	unsigned char lsr;

	lsr = ch943x_port_read(port, CH943X_LSR_REG);
	if (lsr & 0x02) {
		dev_err(port->dev, "Rx Overrun portno = %d, lsr = 0x%2x\n", portno, lsr);
	}

	iir = ch943x_port_read(port, CH943X_IIR_REG);
	if ((iir & CH943X_IIR_NO_INT_BIT) || (one->isopen == false)) {
		dev_vdbg(&s->spi_dev->dev, "%s no int or port not open, quit\n", __func__);
		return;
	}
	iir &= CH943X_IIR_ID_MASK;
	switch (iir) {
	case CH943X_IIR_RDI_SRC:
	case CH943X_IIR_RLSE_SRC:
	case CH943X_IIR_RTOI_SRC:
		ch943x_port_write_spefify(port, 0, CH943X_FIFO_REG, port->line | CH943X_FIFO_RD_BIT);
		rxlen = ch943x_port_read_specify(port, 0, CH943X_FIFOCL_REG);
		rxlen |= ch943x_port_read_specify(port, 0, CH943X_FIFOCH_REG) << 8;
		dev_dbg(&s->spi_dev->dev, "%s rxlen = %d\n", __func__, rxlen);
		if (rxlen)
			ch943x_handle_rx(port, rxlen, iir);
		break;
	case CH943X_IIR_MSI_SRC:
		msr = ch943x_port_read(port, CH943X_MSR_REG);
		s->p[portno].msr_reg = msr;
		uart_handle_cts_change(port, !!(msr & CH943X_MSR_CTS_BIT));
		dev_vdbg(&s->spi_dev->dev, "uart_handle_modem_change = 0x%02x\n", msr);
		break;
	case CH943X_IIR_THRI_SRC:
		mutex_lock(&s->mutex);
		ch943x_handle_tx(port);
		mutex_unlock(&s->mutex);
		break;
	default:
		dev_err(port->dev, "Port %i: Unexpected interrupt: %x", port->line, iir);
		break;
	}
}

static irqreturn_t ch943x_ist_top(int irq, void *dev_id)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t ch943x_ist(int irq, void *dev_id)
{
	struct ch943x_port *s = (struct ch943x_port *)dev_id;
	int i;

	dev_dbg(&s->spi_dev->dev, "ch943x_ist interrupt enter...\n");

	for (i = 0; i < s->uart.nr; ++i)
		ch943x_port_irq(s, i);

	dev_dbg(&s->spi_dev->dev, "%s end\n", __func__);

	return IRQ_HANDLED;
}

static void ch943x_wq_proc(struct work_struct *ws)
{
	struct ch943x_one *one = to_ch943x_one(ws, tx_work);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
	mutex_lock(&s->mutex);
	ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_THRI_BIT, CH943X_IER_THRI_BIT);
	mutex_unlock(&s->mutex);
}

static void ch943x_stop_tx(struct uart_port *port)
{
	struct ch943x_one *one = to_ch943x_one(port, port);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
	schedule_work(&one->stop_tx_work);
}

static void ch943x_stop_rx(struct uart_port *port)
{
	struct ch943x_one *one = to_ch943x_one(port, port);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
	schedule_work(&one->stop_rx_work);
}

static void ch943x_start_tx(struct uart_port *port)
{
	struct ch943x_one *one = to_ch943x_one(port, port);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);

	/* handle rs485 */
	if ((one->rs485.flags & SER_RS485_ENABLED) && (one->rs485.delay_rts_before_send > 0)) {
		mdelay(one->rs485.delay_rts_before_send);
	}
	if (!work_pending(&one->tx_work)) {
		dev_dbg(&s->spi_dev->dev, "%s schedule\n", __func__);
		schedule_work(&one->tx_work);
	}
}

static void ch943x_stop_rx_work_proc(struct work_struct *ws)
{
	struct ch943x_one *one = to_ch943x_one(ws, stop_rx_work);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
	mutex_lock(&s->mutex);
	one->port.read_status_mask &= ~CH943X_LSR_DR_BIT;
	ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_RDI_BIT, 0);
	ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_RLSI_BIT, 0);
	mutex_unlock(&s->mutex);
}

static void ch943x_stop_tx_work_proc(struct work_struct *ws)
{
	struct ch943x_one *one = to_ch943x_one(ws, stop_tx_work);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);
	struct circ_buf *xmit = &one->port.state->xmit;

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
	mutex_lock(&s->mutex);
	/* handle rs485 */
	if (one->rs485.flags & SER_RS485_ENABLED) {
		/* do nothing if current tx not yet completed */
		int lsr = ch943x_port_read(&one->port, CH943X_LSR_REG);
		if (!(lsr & CH943X_LSR_TEMT_BIT)) {
			mutex_unlock(&s->mutex);
			return;
		}
		if (uart_circ_empty(xmit) && (one->rs485.delay_rts_after_send > 0))
			mdelay(one->rs485.delay_rts_after_send);
	}

	ch943x_port_update(&one->port, CH943X_IER_REG, CH943X_IER_THRI_BIT, 0);
	mutex_unlock(&s->mutex);
}

static unsigned int ch943x_tx_empty(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	unsigned int lsr;
	unsigned int result;

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
	lsr = ch943x_port_read(port, CH943X_LSR_REG);
	result = (lsr & CH943X_LSR_THRE_BIT) ? TIOCSER_TEMT : 0;

	return result;
}

static unsigned int ch943x_get_mctrl(struct uart_port *port)
{
	unsigned int status, ret;
	struct ch943x_port *s = dev_get_drvdata(port->dev);

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
	status = s->p[port->line].msr_reg;
	ret = 0;
	if (status & UART_MSR_DCD)
		ret |= TIOCM_CAR;
	if (status & UART_MSR_RI)
		ret |= TIOCM_RNG;
	if (status & UART_MSR_DSR)
		ret |= TIOCM_DSR;
	if (status & UART_MSR_CTS)
		ret |= TIOCM_CTS;

	return ret;
}

static void ch943x_md_proc(struct work_struct *ws)
{
	struct ch943x_one *one = to_ch943x_one(ws, md_work);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);
	unsigned int mctrl = one->port.mctrl;
	unsigned char mcr = 0;

	if (mctrl & TIOCM_RTS) {
		mcr |= UART_MCR_RTS;
	}
	if (mctrl & TIOCM_DTR) {
		mcr |= UART_MCR_DTR;
	}
	if (mctrl & TIOCM_OUT1) {
		mcr |= UART_MCR_OUT1;
	}
	if (mctrl & TIOCM_OUT2) {
		mcr |= UART_MCR_OUT2;
	}
	if (mctrl & TIOCM_LOOP) {
		mcr |= UART_MCR_LOOP;
	}

	mcr |= one->mcr_force;

	dev_dbg(&s->spi_dev->dev, "%s - mcr:0x%x, force:0x%2x\n", __func__, mcr, one->mcr_force);

	ch943x_port_write(&one->port, CH943X_MCR_REG, mcr);
}

static void ch943x_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	struct ch943x_one *one = to_ch943x_one(port, port);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	dev_dbg(&s->spi_dev->dev, "%s - mctrl:0x%x\n", __func__, mctrl);
	schedule_work(&one->md_work);
}

static void ch943x_break_ctl(struct uart_port *port, int break_state)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
	ch943x_port_update(port, CH943X_LCR_REG, CH943X_LCR_TXBREAK_BIT, break_state ? CH943X_LCR_TXBREAK_BIT : 0);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void ch943x_set_termios(struct uart_port *port, struct ktermios *termios, const struct ktermios *old)
#else
static void ch943x_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
#endif
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct ch943x_one *one = to_ch943x_one(port, port);
	unsigned int lcr;
	int baud;
	u8 bParityType;

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);

	/* Word size */
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		lcr = CH943X_LCR_WORD_LEN_5;
		break;
	case CS6:
		lcr = CH943X_LCR_WORD_LEN_6;
		break;
	case CS7:
		lcr = CH943X_LCR_WORD_LEN_7;
		break;
	case CS8:
		lcr = CH943X_LCR_WORD_LEN_8;
		break;
	default:
		lcr = CH943X_LCR_WORD_LEN_8;
		termios->c_cflag &= ~CSIZE;
		termios->c_cflag |= CS8;
		break;
	}

	bParityType = termios->c_cflag & PARENB ?
			      (termios->c_cflag & PARODD ? 1 : 2) + (termios->c_cflag & CMSPAR ? 2 : 0) :
			      0;
	lcr |= CH943X_LCR_PARITY_BIT;
	switch (bParityType) {
	case 0x01:
		lcr |= CH943X_LCR_ODDPARITY_BIT;
		dev_vdbg(&s->spi_dev->dev, "parity = odd\n");
		break;
	case 0x02:
		lcr |= CH943X_LCR_EVENPARITY_BIT;
		dev_vdbg(&s->spi_dev->dev, "parity = even\n");
		break;
	case 0x03:
		lcr |= CH943X_LCR_MARKPARITY_BIT;
		dev_vdbg(&s->spi_dev->dev, "parity = mark\n");
		break;
	case 0x04:
		lcr |= CH943X_LCR_SPACEPARITY_BIT;
		dev_vdbg(&s->spi_dev->dev, "parity = space\n");
		break;
	default:
		lcr &= ~CH943X_LCR_PARITY_BIT;
		dev_vdbg(&s->spi_dev->dev, "parity = none\n");
		break;
	}

	/* Stop bits */
	if (termios->c_cflag & CSTOPB)
		lcr |= CH943X_LCR_STOPLEN_BIT; /* 2 stops */

	/* Set read status mask */
	port->read_status_mask = CH943X_LSR_OE_BIT;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= CH943X_LSR_PE_BIT | CH943X_LSR_FE_BIT;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= CH943X_LSR_BI_BIT;

	/* Set status ignore mask */
	port->ignore_status_mask = 0;
	if (termios->c_iflag & IGNBRK)
		port->ignore_status_mask |= CH943X_LSR_BI_BIT;
	if (!(termios->c_cflag & CREAD))
		port->ignore_status_mask |= CH943X_LSR_BRK_ERROR_MASK;

	/* Update LCR register */
	ch943x_port_write(port, CH943X_LCR_REG, lcr);

	/* Configure flow control */
	if (termios->c_cflag & CRTSCTS) {
		dev_vdbg(&s->spi_dev->dev, "ch943x_set_termios enable rts/cts\n");
		ch943x_port_update(port, CH943X_MCR_REG, CH943X_MCR_AFE | CH943X_MCR_RTS_BIT,
				   CH943X_MCR_AFE | CH943X_MCR_RTS_BIT);
		one->mcr_force |= CH943X_MCR_AFE | CH943X_MCR_RTS_BIT;
		// add on 20200608 suppose cts status is always valid here
		uart_handle_cts_change(port, 1);
	} else {
		dev_vdbg(&s->spi_dev->dev, "ch943x_set_termios disable rts/cts\n");
		ch943x_port_update(port, CH943X_MCR_REG, CH943X_MCR_AFE, 0);
		one->mcr_force &= ~(CH943X_MCR_AFE | CH943X_MCR_RTS_BIT);
	}

	/* Get baud rate generator configuration */
	baud = uart_get_baud_rate(port, termios, old, port->uartclk / 16 / 0xffff, port->uartclk / 16 * 24);
	/* Setup baudrate generator */
	baud = ch943x_set_baud(port, baud);
	/* Update timeout according to new baud rate */
	uart_update_timeout(port, termios->c_cflag, baud);
}

static void ch943x_config_rs485(struct uart_port *port, struct serial_rs485 *rs485)
{
	struct ch943x_one *one = to_ch943x_one(port, port);
	struct ch943x_port *s = dev_get_drvdata(one->port.dev);

	one->rs485 = *rs485;
	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
	if (one->rs485.flags & SER_RS485_ENABLED) {
		s->reg485 |= 1 << port->line;
	} else {
		s->reg485 &= ~(1 << port->line);
	}
	ch943x_port_write_spefify(port, 0, CH943X_RS485_REG, s->reg485);
}

static int ch943x_ioctl(struct uart_port *port, unsigned int cmd, unsigned long arg)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	int rv = 0;
	struct serial_rs485 rs485;
	u16 inarg;
	u16 __user *argval = (u16 __user *)arg;
	u8 val;

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);

	switch (cmd) {
	case IOCTL_CMD_SRS485:
		if (copy_from_user(&rs485, (void __user *)arg, sizeof(rs485)))
			return -EFAULT;

		ch943x_config_rs485(port, &rs485);
		break;
	case IOCTL_CMD_GRS485:
		if (copy_to_user((void __user *)arg, &(to_ch943x_one(port, port)->rs485), sizeof(rs485)))
			return -EFAULT;
		break;
	case IOCTL_CMD_GPIOENABLE:
		if (get_user(inarg, argval))
			return -EFAULT;
		ch943x_port_write_spefify(port, 0, CH943X_GPIOEN_REG + (inarg >> 8), inarg);
		break;
	case IOCTL_CMD_GPIODIR:
		if (get_user(inarg, argval))
			return -EFAULT;
		ch943x_port_write_spefify(port, 0, CH943X_GPIODIR_REG + (inarg >> 8), inarg);
		break;
	case IOCTL_CMD_GPIOPULLUP:
		if (get_user(inarg, argval))
			return -EFAULT;
		ch943x_port_write_spefify(port, 0, CH943X_GPIOPU_REG + (inarg >> 8), inarg);
		ch943x_port_write_spefify(port, 0, CH943X_GPIOPD_REG + (inarg >> 8), ~inarg);
		break;
	case IOCTL_CMD_GPIOPULLDOWN:
		if (get_user(inarg, argval))
			return -EFAULT;
		ch943x_port_write_spefify(port, 0, CH943X_GPIOPD_REG + (inarg >> 8), inarg);
		ch943x_port_write_spefify(port, 0, CH943X_GPIOPU_REG + (inarg >> 8), ~inarg);
		break;
	case IOCTL_CMD_GPIOSET:
		if (get_user(inarg, argval))
			return -EFAULT;
		ch943x_port_write_spefify(port, 0, CH943X_GPIOVAL_REG + (inarg >> 8), inarg);
		break;
	case IOCTL_CMD_GPIOGET:
		if (get_user(inarg, argval))
			return -EFAULT;
		val = ch943x_port_read_specify(port, 0, CH943X_GPIOVAL_REG + (inarg >> 8));
		if (put_user(val, argval))
			return -EFAULT;
		break;
	default:
		rv = -ENOIOCTLCMD;
		break;
	}

	return rv;
}

static int ch943x_startup(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct ch943x_one *one = to_ch943x_one(port, port);
	u8 val;

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);

	/* kmalloc spi transfer buffer when startup */
	one->rxbuf = kmalloc(2048, GFP_KERNEL);
	if (!one->rxbuf) {
		dev_err(&s->spi_dev->dev, "kmalloc rxbuf failed\n");
		return -1;
	}

	one->txbuf = kmalloc(2048, GFP_KERNEL);
	if (!one->txbuf) {
		dev_err(&s->spi_dev->dev, "kmalloc txbuf failed\n");
		return -1;
	}

	/* Reset FIFOs and configure RX-FIFO levels to 8 */
	ch943x_port_write(port, CH943X_FCR_REG,
			  CH943X_FCR_RXRESET_BIT | CH943X_FCR_TXRESET_BIT | CH943X_FCR_RXLVLH_BIT |
				  CH943X_FCR_FIFO_BIT);
	udelay(2000);

	/* Now, initialize the UART */
	ch943x_port_write(port, CH943X_LCR_REG, CH943X_LCR_WORD_LEN_8);

	/* Enable RX, TX, CTS change interrupts */
	val = CH943X_IER_RDI_BIT | CH943X_IER_RLSI_BIT | CH943X_IER_MSI_BIT;
	ch943x_port_write(port, CH943X_IER_REG, val);

	/* Enable Uart interrupts */
	ch943x_port_write(port, CH943X_MCR_REG, CH943X_MCR_OUT2);
	one->mcr_force = CH943X_MCR_OUT2;
	one->isopen = true;

	return 0;
}

static void ch943x_shutdown(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	struct ch943x_one *one = to_ch943x_one(port, port);

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);

	/* Disable all interrupts */
	ch943x_port_write(port, CH943X_IER_REG, 0);
	ch943x_port_write(port, CH943X_MCR_REG, 0);

	if (one->rxbuf)
		kfree(one->rxbuf);
	if (one->txbuf)
		kfree(one->txbuf);
	one->mcr_force = 0;
	one->isopen = false;
}

static const char *ch943x_type(struct uart_port *port)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);
	return (port->type == PORT_SC16IS7XX) ? s->devtype->name : NULL;
}

static int ch943x_request_port(struct uart_port *port)
{
	/* Do nothing */
	return 0;
}

static void ch943x_config_port(struct uart_port *port, int flags)
{
	if (flags & UART_CONFIG_TYPE)
		port->type = PORT_SC16IS7XX;
}

static int ch943x_verify_port(struct uart_port *port, struct serial_struct *s)
{
	if ((s->type != PORT_UNKNOWN) && (s->type != PORT_SC16IS7XX))
		return -EINVAL;
	if (s->irq != port->irq)
		return -EINVAL;

	return 0;
}

static void ch943x_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
	struct ch943x_port *s = dev_get_drvdata(port->dev);

	dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
}

static void ch943x_null_void(struct uart_port *port)
{
	/* Do nothing */
}

static void ch943x_enable_ms(struct uart_port *port)
{
	/* Do nothing */
}

static const struct uart_ops ch943x_ops = {
	.tx_empty = ch943x_tx_empty,
	.set_mctrl = ch943x_set_mctrl,
	.get_mctrl = ch943x_get_mctrl,
	.stop_tx = ch943x_stop_tx,
	.start_tx = ch943x_start_tx,
	.stop_rx = ch943x_stop_rx,
	.break_ctl = ch943x_break_ctl,
	.startup = ch943x_startup,
	.shutdown = ch943x_shutdown,
	.set_termios = ch943x_set_termios,
	.type = ch943x_type,
	.request_port = ch943x_request_port,
	.release_port = ch943x_null_void,
	.config_port = ch943x_config_port,
	.verify_port = ch943x_verify_port,
	.ioctl = ch943x_ioctl,
	.enable_ms = ch943x_enable_ms,
	.pm = ch943x_pm,
};

static int ch943x_probe(struct spi_device *spi, struct ch943x_devtype *devtype, int irq, unsigned long flags)
{
	unsigned long freq;
	int i, ret;
	struct ch943x_port *s;
	struct device *dev = &spi->dev;
	u8 clkdiv = 13;
	u8 spi_contmode_reg;

	/* Alloc port structure */
	s = devm_kzalloc(dev, sizeof(*s) + sizeof(struct ch943x_one) * devtype->nr_uart, GFP_KERNEL);
	if (!s) {
		dev_err(dev, "Error allocating port structure\n");
		return -ENOMEM;
	}
	freq = 32 * 1000000 * 15 / clkdiv;
	s->devtype = devtype;
	dev_set_drvdata(dev, s);
	s->spi_dev = spi;

	/* Register UART driver */
	s->uart.owner = THIS_MODULE;
	s->uart.dev_name = "ttyWCH";
	s->uart.nr = devtype->nr_uart;
	ret = uart_register_driver(&s->uart);
	if (ret) {
		dev_err(dev, "Registering UART driver failed\n");
		goto out_clk;
	}

	mutex_init(&s->mutex);
	mutex_init(&s->mutex_bus_access);

	for (i = 0; i < devtype->nr_uart; i++) {
		/* Initialize port data */
		s->p[i].port.line = i;
		s->p[i].port.dev = dev;
		s->p[i].port.irq = irq;
		s->p[i].port.type = PORT_SC16IS7XX;
		s->p[i].port.fifosize = CH943X_FIFO_SIZE;
		s->p[i].port.flags = UPF_FIXED_TYPE | UPF_LOW_LATENCY;
		s->p[i].port.iotype = UPIO_PORT;
		s->p[i].port.uartclk = freq;
		s->p[i].port.ops = &ch943x_ops;
		/* Disable all interrupts */
		ch943x_port_write(&s->p[i].port, CH943X_IER_REG, 0);
		/* Disable uart interrupts */
		ch943x_port_write(&s->p[i].port, CH943X_MCR_REG, 0);

		s->p[i].msr_reg = ch943x_port_read(&s->p[i].port, CH943X_MSR_REG);

		/* Initialize queue for start TX */
		INIT_WORK(&s->p[i].tx_work, ch943x_wq_proc);
		/* Initialize queue for changing mode */
		INIT_WORK(&s->p[i].md_work, ch943x_md_proc);

		INIT_WORK(&s->p[i].stop_rx_work, ch943x_stop_rx_work_proc);
		INIT_WORK(&s->p[i].stop_tx_work, ch943x_stop_tx_work_proc);

		ret = ch943x_scr_test(&s->p[i].port);
		if (ret)
			goto out;

		/* Register port */
		uart_add_one_port(&s->uart, &s->p[i].port);
	}

	ret = ch943x_port_read_multi(&s->p[0].port, 0, CH943X_CHIP_VER_REG, s->ver, 4);
	if (ret)
		goto out;
	if ((s->ver[3] != 0x5A) || (s->ver[2] != (s->ver[0] + s->ver[1]))) {
		s->spi_contmode = false;
		s->ver[0] = 0;
		s->ver[1] = 1;
	} else {
		if (s->ver[1] & 0xf0) {
			dev_err(&s->spi_dev->dev, "Current driver does not support this chip version.\n");
			goto out;
		}

		if (spi->max_speed_hz > 2000000) {
			dev_err(&s->spi_dev->dev,
				"SPI continuous transmission mode can only be enabled when the SPI clock does not exceed 2M.\n");
			goto out;
		}
		spi_contmode_reg = ch943x_port_read(&s->p[0].port, CH943X_SPI_CONT_MODE_REG);
		ch943x_port_write(&s->p[0].port, CH943X_SPI_CONT_MODE_REG, CH943X_SPI_CONTE_BIT | spi_contmode_reg);
		s->spi_contmode = true;
	}
	dev_info(dev, "%s - firmware version: V%d.%d\n", __func__, s->ver[1], s->ver[0]);

	/* Init UART Clock */
	ch943x_port_write(&s->p[0].port, CH943X_CLK_REG, CH943X_CLK_EXT_BIT | CH943X_CLK_PLL_BIT | clkdiv);

	ret = devm_request_threaded_irq(dev, irq, ch943x_ist_top, ch943x_ist, IRQF_ONESHOT | flags, dev_name(dev), s);
	// flags, dev_name(dev), s);
	mdelay(200);
	dev_dbg(dev, "%s - devm_request_threaded_irq =%d result:%d\n", __func__, irq, ret);
	g_ch943x_port = s;

	if (!ret)
		return 0;

out:
	mutex_destroy(&s->mutex);

	uart_unregister_driver(&s->uart);

out_clk:
	if (!IS_ERR(s->clk))
		/*clk_disable_unprepare(s->clk)*/;

	return ret;
}
static struct attribute_group ch9434_attribute_group;
static int ch943x_remove(struct device *dev)
{
	struct ch943x_port *s = dev_get_drvdata(dev);
	int i;

	dev_dbg(dev, "%s\n", __func__);

	devm_free_irq(dev, s->p[0].port.irq, s);

	for (i = 0; i < s->uart.nr; i++) {
		cancel_work_sync(&s->p[i].tx_work);
		cancel_work_sync(&s->p[i].md_work);
		cancel_work_sync(&s->p[i].stop_rx_work);
		cancel_work_sync(&s->p[i].stop_tx_work);
		uart_remove_one_port(&s->uart, &s->p[i].port);
	}

	mutex_destroy(&s->mutex);
	mutex_destroy(&s->mutex_bus_access);
	uart_unregister_driver(&s->uart);
	if (!IS_ERR(s->clk))
		/*clk_disable_unprepare(s->clk)*/;

	sysfs_remove_group(&dev->kobj, &ch9434_attribute_group);

	return 0;
}

static const struct of_device_id __maybe_unused ch943x_dt_ids[] = {
	{
		.compatible = "wch,ch943x",
		.data = &ch943x_devtype,
	},
	{},
};
MODULE_DEVICE_TABLE(of, ch943x_dt_ids);

static ssize_t reg_dump_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct uart_port *port;
	int i;

	dev_info(dev, "reg_dump_show");

	for (i = 0; i < 4; i++) {
		port = &g_ch943x_port->p[i].port;
		ch943x_dump_register(port);
	}

	return 0;
}

static ssize_t reg_dump_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static DEVICE_ATTR(reg_dump, S_IRUGO | S_IWUSR, reg_dump_show, reg_dump_store);

static struct attribute *ch9434_attributes[] = { &dev_attr_reg_dump.attr, NULL };

static struct attribute_group ch9434_attribute_group = { .attrs = ch9434_attributes };

int ch9434_create_sysfs(struct spi_device *spi)
{
	int err;

	err = sysfs_create_group(&spi->dev.kobj, &ch9434_attribute_group);
	if (err != 0) {
		dev_err(&spi->dev, "sysfs_create_group() failed!!");
		sysfs_remove_group(&spi->dev.kobj, &ch9434_attribute_group);
		return -EIO;
	}

	dev_info(&spi->dev, "sysfs_create_group() succeeded!!");

	return err;
}

#ifdef USE_SPI_MODE
static int ch943x_spi_probe(struct spi_device *spi)
{
	struct ch943x_devtype *devtype = &ch943x_devtype;
	unsigned long flags = IRQF_TRIGGER_LOW;
	int ret;
	u32 save;

	dev_dbg(&spi->dev, "gpio_to_irq:%d, spi->irq:%d\n", gpio_to_irq(GPIO_NUMBER), spi->irq);
	save = spi->mode;
	spi->mode |= SPI_MODE_3;

	if (spi_setup(spi) < 0) {
		spi->mode = save;
	} else {
		dev_dbg(&spi->dev, "change to SPI MODE 3!\n");
	}

/* if your platform supports acquire irq number from dts */
#ifdef USE_IRQ_FROM_DTS
	ret = ch943x_probe(spi, devtype, spi->irq, flags);
	if (ret) {
		dev_err(&spi->dev, "ch943x_probe error\n");
		goto out;
	}
#else
	ret = devm_gpio_request(&spi->dev, GPIO_NUMBER, "gpioint");
	if (ret) {
		dev_err(&spi->dev, "gpio_request\n");
		goto out;
	}
	ret = gpio_direction_input(GPIO_NUMBER);
	if (ret) {
		dev_err(&spi->dev, "gpio_direction_input\n");
		goto out;
	}
	irq_set_irq_type(gpio_to_irq(GPIO_NUMBER), flags);
	ret = ch943x_probe(spi, devtype, gpio_to_irq(GPIO_NUMBER), flags);
	if (ret) {
		dev_err(&spi->dev, "ch943x_probe error\n");
		goto out;
	}
#endif

	ch9434_create_sysfs(spi);

out:
	return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
static void ch943x_spi_remove(struct spi_device *spi)
#else
static int ch943x_spi_remove(struct spi_device *spi)
#endif
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
	ch943x_remove(&spi->dev);
#else
	return ch943x_remove(&spi->dev);
#endif
}

static struct spi_driver ch943x_spi_uart_driver = {
    .driver =
        {
                 .name = CH943X_NAME_SPI,
                 .bus = &spi_bus_type,
                 .owner = THIS_MODULE,
                 .of_match_table = of_match_ptr(ch943x_dt_ids),
                 },
    .probe = ch943x_spi_probe,
    .remove = ch943x_spi_remove,
};

MODULE_ALIAS("spi:ch943x");
#endif

#ifdef USE_SPI_MODE
static int __init ch943x_init(void)
{
	printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");
	printk(KERN_INFO KBUILD_MODNAME ": " VERSION_DESC "\n");
	return spi_register_driver(&ch943x_spi_uart_driver);
}
module_init(ch943x_init);

static void __exit ch943x_exit(void)
{
	printk(KERN_INFO KBUILD_MODNAME ": "
					"ch943x driver exit.\n");
	spi_unregister_driver(&ch943x_spi_uart_driver);
}
module_exit(ch943x_exit);
#endif

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
