// SPDX-License-Identifier: GPL-2.0+
/*
 * SPI to UART driver for chip CH432, etc.
 *
 * Copyright (C) 2026 Nanjing Qinheng Microelectronics Co., Ltd.
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
 * V1.1 - fix bugs in ch43x_start_tx
 * V1.2 - fix high baud rates setting bugs
 *      - change fifo trigger to 8
 *      - add receive timeout handling
 *      - add support of hardflow setting
 * V1.3 - modify rs485 configuration in ioctl, add support for sysfs debug
 * V1.4 - add support multiple chips, modify sysfs debug
 */

#define DEBUG
#define VERBOSE_DEBUG

#undef DEBUG
#undef VERBOSE_DEBUG

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/serial_reg.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/uaccess.h>
#include <linux/irq.h>
#include <linux/fs.h>
#include <linux/fs_struct.h>
#include <linux/proc_fs.h>
#include "linux/version.h"

#define DRIVER_AUTHOR "WCH"
#define DRIVER_DESC   "SPI serial driver for ch432."
#define VERSION_DESC  "V1.4 On 2026.04"

/* external crystal freq */
#define CRYSTAL_FREQ   22118400
#define CH43X_NAME     "ch43x"
#define CH43X_NAME_SPI "ch43x_spi"

#define USE_IRQ_FROM_DTS
#define CH43X_MAX_CNT 8
#define GPIO_NUMBER   0

/* CH43X register definitions */
#define CH43X_RHR_REG (0x00) /* RX FIFO */
#define CH43X_THR_REG (0x00) /* TX FIFO */
#define CH43X_IER_REG (0x01) /* Interrupt enable */
#define CH43X_IIR_REG (0x02) /* Interrupt Identification */
#define CH43X_FCR_REG (0x02) /* FIFO control */
#define CH43X_LCR_REG (0x03) /* Line Control */
#define CH43X_MCR_REG (0x04) /* Modem Control */
#define CH43X_LSR_REG (0x05) /* Line Status */
#define CH43X_MSR_REG (0x06) /* Modem Status */
#define CH43X_SPR_REG (0x07) /* Scratch Pad */

/* Special Register set: Only if (LCR[7] == 1) */
#define CH43X_DLL_REG (0x00) /* Divisor Latch Low */
#define CH43X_DLH_REG (0x01) /* Divisor Latch High */

/* IER register bits */
#define CH43X_IER_RDI_BIT  (1 << 0) /* Enable RX data interrupt */
#define CH43X_IER_THRI_BIT (1 << 1) /* Enable TX holding register interrupt */
#define CH43X_IER_RLSI_BIT (1 << 2) /* Enable RX line status interrupt */
#define CH43X_IER_MSI_BIT  (1 << 3) /* Enable Modem status interrupt */

/* IER enhanced register bits */
#define CH43X_IER_RESET_BIT    (1 << 7) /* Enable Soft reset */
#define CH43X_IER_LOWPOWER_BIT (1 << 6) /* Enable low power mode */
#define CH43X_IER_SLEEP_BIT    (1 << 5) /* Enable sleep mode */
#define CH43X_IER_CK2X_BIT     (1 << 5) /* Enable clk * 2 */

/* FCR register bits */
#define CH43X_FCR_FIFO_BIT    (1 << 0) /* Enable FIFO */
#define CH43X_FCR_RXRESET_BIT (1 << 1) /* Reset RX FIFO */
#define CH43X_FCR_TXRESET_BIT (1 << 2) /* Reset TX FIFO */
#define CH43X_FCR_RXLVLL_BIT  (1 << 6) /* RX Trigger level LSB */
#define CH43X_FCR_RXLVLH_BIT  (1 << 7) /* RX Trigger level MSB */

/* IIR register bits */
#define CH43X_IIR_NO_INT_BIT (1 << 0) /* No interrupts pending */
#define CH43X_IIR_ID_MASK    0x0e     /* Mask for the interrupt ID */
#define CH43X_IIR_THRI_SRC   0x02     /* TX holding register empty */
#define CH43X_IIR_RDI_SRC    0x04     /* RX data interrupt */
#define CH43X_IIR_RLSE_SRC   0x06     /* RX line status error */
#define CH43X_IIR_RTOI_SRC   0x0c     /* RX time-out interrupt */
#define CH43X_IIR_MSI_SRC    0x00     /* Modem status interrupt */

/* LCR register bits */
#define CH43X_LCR_LENGTH0_BIT     (1 << 0) /* Word length bit 0 */
#define CH43X_LCR_LENGTH1_BIT     (1 << 1)
#define CH43X_LCR_STOPLEN_BIT     (1 << 2)
#define CH43X_LCR_PARITY_BIT      (1 << 3) /* Parity bit enable */
#define CH43X_LCR_ODDPARITY_BIT   (0)      /* Odd parity bit enable */
#define CH43X_LCR_EVENPARITY_BIT  (1 << 4) /* Even parity bit enable */
#define CH43X_LCR_MARKPARITY_BIT  (1 << 5) /* Mark parity bit enable */
#define CH43X_LCR_SPACEPARITY_BIT (3 << 4) /* Space parity bit enable */

#define CH43X_LCR_TXBREAK_BIT (1 << 6) /* TX break enable */
#define CH43X_LCR_DLAB_BIT    (1 << 7) /* Divisor Latch enable */
#define CH43X_LCR_WORD_LEN_5  (0x00)
#define CH43X_LCR_WORD_LEN_6  (0x01)
#define CH43X_LCR_WORD_LEN_7  (0x02)
#define CH43X_LCR_WORD_LEN_8  (0x03)
#define CH43X_LCR_CONF_MODE_A CH43X_LCR_DLAB_BIT /* Special reg set */

/* MCR register bits */
#define CH43X_MCR_DTR_BIT  (1 << 0) /* DTR complement */
#define CH43X_MCR_RTS_BIT  (1 << 1) /* RTS complement */
#define CH43X_MCR_OUT1     (1 << 2) /* OUT1 */
#define CH43X_MCR_OUT2     (1 << 3) /* OUT2 */
#define CH43X_MCR_LOOP_BIT (1 << 4) /* Enable loopback test mode */
#define CH43X_MCR_AFE      (1 << 5) /* Enable Hardware Flow control */

/* LSR register bits */
#define CH43X_LSR_DR_BIT         (1 << 0) /* Receiver data ready */
#define CH43X_LSR_OE_BIT         (1 << 1) /* Overrun Error */
#define CH43X_LSR_PE_BIT         (1 << 2) /* Parity Error */
#define CH43X_LSR_FE_BIT         (1 << 3) /* Frame Error */
#define CH43X_LSR_BI_BIT         (1 << 4) /* Break Interrupt */
#define CH43X_LSR_BRK_ERROR_MASK 0x1E     /* BI, FE, PE, OE bits */
#define CH43X_LSR_THRE_BIT       (1 << 5) /* TX holding register empty */
#define CH43X_LSR_TEMT_BIT       (1 << 6) /* Transmitter empty */
#define CH43X_LSR_FIFOE_BIT      (1 << 7) /* Fifo Error */

/* MSR register bits */
#define CH43X_MSR_DCTS_BIT   (1 << 0) /* Delta CTS Clear To Send */
#define CH43X_MSR_DDSR_BIT   (1 << 1) /* Delta DSR Data Set Ready */
#define CH43X_MSR_DRI_BIT    (1 << 2) /* Delta RI Ring Indicator */
#define CH43X_MSR_DCD_BIT    (1 << 3) /* Delta CD Carrier Detect */
#define CH43X_MSR_CTS_BIT    (1 << 4) /* CTS */
#define CH43X_MSR_DSR_BIT    (1 << 5) /* DSR */
#define CH43X_MSR_RI_BIT     (1 << 6) /* RI */
#define CH43X_MSR_CD_BIT     (1 << 7) /* CD */
#define CH43X_MSR_DELTA_MASK 0x0F     /* Any of the delta bits! */

/* Misc definitions */
#define CH43X_FIFO_SIZE (16)
#define CH43X_REG_SHIFT 2
#define REGS_BUFSIZE 4096

struct ch43x_devtype {
    char name[10];
    int nr_uart;
};

struct ch43x_one {
    struct uart_port port;
    struct work_struct tx_work;
    struct work_struct md_work;
    struct work_struct stop_rx_work;
    struct work_struct stop_tx_work;
    struct serial_rs485 rs485;
    unsigned char msr_reg;
    unsigned char mcr_force;
};

struct ch43x_port {
    struct uart_driver uart;
    struct ch43x_devtype *devtype;
    int minor;
    struct mutex mutex;
    struct mutex mutex_bus_access;
    struct clk *clk;
    struct spi_device *spi_dev;
    char proc_file_name[16];
    unsigned char buf[65536];
    struct ch43x_one p[0];
};

static struct ch43x_port *ch43x_table[CH43X_MAX_CNT];
static DEFINE_MUTEX(ch43x_minors_lock);

static int ch43x_alloc_minor(struct ch43x_port *s)
{
    int minor;

    mutex_lock(&ch43x_minors_lock);
    for (minor = 0; minor < CH43X_MAX_CNT; minor++) {
        if (!ch43x_table[minor]) {
            ch43x_table[minor] = s;
            break;
        }
    }
    mutex_unlock(&ch43x_minors_lock);

    return minor;
}

static void ch43x_release_minor(struct ch43x_port *s)
{
    mutex_lock(&ch43x_minors_lock);
    ch43x_table[s->minor] = NULL;
    mutex_unlock(&ch43x_minors_lock);
}

#define to_ch43x_one(p, e) ((container_of((p), struct ch43x_one, e)))

static int ch43x_reg_write(struct ch43x_port *s, u8 cmd, u8 val)
{
    uint8_t spi_buf[2];
    int status;

    spi_buf[0] = cmd;
    spi_buf[1] = val;

    mutex_lock(&s->mutex_bus_access);
    status = spi_write(s->spi_dev, spi_buf, 2);
    mutex_unlock(&s->mutex_bus_access);
    if (status < 0) {
        return status;
    }

    return 0;
}

static int ch43x_port_write(struct uart_port *port, u8 reg, u8 val)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    uint8_t cmd;
    int status;

    cmd = 0x02 | ((reg + port->line * 0x08) << CH43X_REG_SHIFT);

    status = ch43x_reg_write(s, cmd, val);
    if (status < 0) {
        dev_err(&s->spi_dev->dev, "Failed to ch43x_port_write Err_code %d\n", status);
        return status;
    }
    dev_dbg(&s->spi_dev->dev, "%s - cmd:%02x[u%d reg:%02x], data:%02x\n", __func__, cmd, port->line, reg, val);

    return 0;
}

static uint8_t ch43x_reg_read(struct ch43x_port *s, u8 cmd)
{
    uint8_t txbuf[2] = {0};
    uint8_t rxbuf[2] = {0};
    struct spi_message m;
    struct spi_transfer xfer[2] = {};
    uint8_t result;
    int status;

    txbuf[0] = cmd;
    txbuf[1] = 0xff;

    xfer[0].tx_buf = txbuf;
    xfer[0].rx_buf = rxbuf;
    xfer[0].len = 2;

    spi_message_init(&m);
    spi_message_add_tail(&xfer[0], &m);
    mutex_lock(&s->mutex_bus_access);
    status = spi_sync(s->spi_dev, &m);
    mutex_unlock(&s->mutex_bus_access);
    if (status < 0) {
        return status;
    }
    result = rxbuf[1];

    return result;
}

static uint8_t ch43x_port_read(struct uart_port *port, u8 reg)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    uint8_t cmd;
    uint8_t result;

    cmd = 0xFD & ((reg + port->line * 0x08) << CH43X_REG_SHIFT);

    result = ch43x_reg_read(s, cmd);
    if (result < 0) {
        dev_err(&s->spi_dev->dev, "Failed to ch43x_port_read error code %d\n", result);
        return result;
    }
    dev_dbg(&s->spi_dev->dev, "%s - cmd:%02x[u%d reg:%02x], data:%02x\n", __func__, cmd, port->line, reg, result);

    return result;
}

static u8 ch43x_port_read_specify(struct uart_port *port, u8 portnum, u8 reg)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    unsigned char cmd;
    u8 result;

    cmd = 0xFD & ((reg + portnum * 0x08) << CH43X_REG_SHIFT);

    result = ch43x_reg_read(s, cmd);
    if (result < 0) {
        dev_err(&s->spi_dev->dev, "Failed to ch43x_port_read error code %d\n", result);
        return result;
    }
    dev_dbg(&s->spi_dev->dev, "%s - cmd:%2x[u%d reg:%02x], data:%2x\n", __func__, cmd, port->line, reg, result);

    return result;
}
static int ch43x_port_write_spefify(struct uart_port *port, u8 portnum, u8 reg, u8 val)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    unsigned char cmd;
    int status;

    cmd = 0x02 | ((reg + portnum * 0x08) << CH43X_REG_SHIFT);

    status = ch43x_reg_write(s, cmd, val);
    if (status < 0) {
        dev_err(&s->spi_dev->dev, "Failed to ch43x_port_write Err_code %d\n", status);
        return status;
    }
    dev_dbg(&s->spi_dev->dev, "%s - cmd:%2x[u%d reg:%02x], data:%2x\n", __func__, cmd, port->line, reg, val);

    return 0;
}

static void ch43x_port_update(struct uart_port *port, u8 reg, u8 mask, u8 val)
{
    unsigned int tmp;

    tmp = ch43x_port_read(port, reg);
    tmp &= ~mask;
    tmp |= val & mask;
    ch43x_port_write(port, reg, tmp);
}

static void ch43x_port_update_specify(struct uart_port *port, u8 portnum, u8 reg, u8 mask, u8 val)
{
    unsigned int tmp;

    tmp = ch43x_port_read_specify(port, portnum, reg);
    tmp &= ~mask;
    tmp |= val & mask;
    ch43x_port_write_spefify(port, portnum, reg, tmp);
}

void ch43x_raw_write(struct uart_port *port, const void *reg, unsigned char *buf, int len)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    ssize_t status;
    struct spi_message m;
    struct spi_transfer t[2] = {0};

    t[0].tx_buf = reg;
    t[0].len = 1;
    t[1].tx_buf = buf;
    t[1].len = len;

    mutex_lock(&s->mutex_bus_access);
    spi_message_init(&m);
    spi_message_add_tail(&t[0], &m);
    spi_message_add_tail(&t[1], &m);
    status = spi_sync(s->spi_dev, &m);
    mutex_unlock(&s->mutex_bus_access);
    if (status < 0) {
        dev_err(&s->spi_dev->dev, "Failed to ch43x_raw_write Err_code %ld\n", (unsigned long)status);
    }
}

static struct ch43x_devtype ch43x_devtype = {
    .name = "CH43X",
    .nr_uart = 2,
};

static int ch43x_set_baud(struct uart_port *port, int baud)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    u8 lcr;
    unsigned long clk = port->uartclk;
    unsigned long div;

    dev_dbg(&s->spi_dev->dev, "%s - %d\n", __func__, baud);

    /* when use clock multipication */
    div = clk / 16 / baud;

    lcr = ch43x_port_read(port, CH43X_LCR_REG);

    /* Open the LCR divisors for configuration */
    ch43x_port_write(port, CH43X_LCR_REG, CH43X_LCR_CONF_MODE_A);

    /* Write the new divisor */
    ch43x_port_write(port, CH43X_DLH_REG, div / 256);
    ch43x_port_write(port, CH43X_DLL_REG, div % 256);

    /* Put LCR back to the normal mode */
    ch43x_port_write(port, CH43X_LCR_REG, lcr);

    return DIV_ROUND_CLOSEST(clk / 16, div);
}

static int ch43x_spi_test(struct ch43x_port *s)
{
    int ret, i;

    dev_dbg(&s->spi_dev->dev, "******SPI Test Start******\n");

    for (i = 0; i < 2; i++) {
        ret = ch43x_reg_write(s, 0x02 | ((CH43X_SPR_REG + i * 0x08) << CH43X_REG_SHIFT), 0x55);
        if (ch43x_reg_read(s, 0xFD & ((CH43X_SPR_REG + i * 0x08) << CH43X_REG_SHIFT)) != 0x55) {
            dev_err(&s->spi_dev->dev, "%s Failed.", __func__);
            return -1;
        }
        ret = ch43x_reg_write(s, 0x02 | ((CH43X_SPR_REG + i * 0x08) << CH43X_REG_SHIFT), 0xAA);
        if (ch43x_reg_read(s, 0xFD & ((CH43X_SPR_REG + i * 0x08) << CH43X_REG_SHIFT)) != 0xAA) {
            dev_err(&s->spi_dev->dev, "%s Failed.", __func__);
            return -1;
        }
    }

    dev_dbg(&s->spi_dev->dev, "******SPI SPR Test End******\n");
    return 0;
}

static void ch43x_handle_rx(struct uart_port *port, unsigned int iir)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    unsigned int lsr = 0, ch, flag, bytes_read = 0;
    bool read_lsr = (iir == CH43X_IIR_RLSE_SRC) ? true : false;

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
    /* Only read lsr if there are possible errors in FIFO */
    if (read_lsr) {
        lsr = ch43x_port_read(port, CH43X_LSR_REG);
        /* No errors left in FIFO */
        if (!(lsr & CH43X_LSR_FIFOE_BIT))
            read_lsr = false;
    } else
        lsr = 0;

    /* At lest one error left in FIFO */
    if (read_lsr) {
        ch = ch43x_port_read(port, CH43X_RHR_REG);
        bytes_read = 1;

        goto ch_handler;
    } else {
        while (((lsr = ch43x_port_read(port, CH43X_LSR_REG)) & CH43X_LSR_DR_BIT) == 0)
            ;

        do {
            if (likely(lsr & CH43X_LSR_DR_BIT)) {
                ch = ch43x_port_read(port, CH43X_RHR_REG);
                bytes_read++;
            } else
                break;
        ch_handler:
            flag = TTY_NORMAL;
            port->icount.rx++;

            if (unlikely(lsr & CH43X_LSR_BRK_ERROR_MASK)) {
                if (lsr & CH43X_LSR_BI_BIT) {
                    lsr &= ~(CH43X_LSR_FE_BIT | CH43X_LSR_PE_BIT);
                    port->icount.brk++;
                    if (uart_handle_break(port))
                        goto ignore_char;
                } else if (lsr & CH43X_LSR_PE_BIT)
                    port->icount.parity++;
                else if (lsr & CH43X_LSR_FE_BIT)
                    port->icount.frame++;
                else if (lsr & CH43X_LSR_OE_BIT)
                    port->icount.overrun++;

                lsr &= port->read_status_mask;
                if (lsr & CH43X_LSR_BI_BIT)
                    flag = TTY_BREAK;
                else if (lsr & CH43X_LSR_PE_BIT)
                    flag = TTY_PARITY;
                else if (lsr & CH43X_LSR_FE_BIT)
                    flag = TTY_FRAME;

                if (lsr & CH43X_LSR_OE_BIT)
                    dev_err(&s->spi_dev->dev, "%s - overrun detect\n", __func__);
            }

            if (uart_handle_sysrq_char(port, ch)) {
                goto ignore_char;
            }
            uart_insert_char(port, lsr, CH43X_LSR_OE_BIT, ch, flag);
        ignore_char:
            lsr = ch43x_port_read(port, CH43X_LSR_REG);
        } while ((lsr & CH43X_LSR_DR_BIT));
    }
    dev_dbg(&s->spi_dev->dev, "%s-bytes_read:%d\n", __func__, bytes_read);
    tty_flip_buffer_push(&port->state->port);
}

static void ch43x_handle_tx(struct uart_port *port)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    struct circ_buf *xmit = &port->state->xmit;
    unsigned int txlen, to_send, i;
    unsigned char thr_reg;

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
    /* xon/xoff char */
    if (unlikely(port->x_char)) {
        ch43x_port_write(port, CH43X_THR_REG, port->x_char);
        port->icount.tx++;
        port->x_char = 0;
        return;
    }

    if (uart_circ_empty(xmit) || uart_tx_stopped(port)) {
        dev_dbg(&s->spi_dev->dev, "ch43x_handle_tx stopped\n");
        // add on 20200608
        ch43x_port_update(port, CH43X_IER_REG, CH43X_IER_THRI_BIT, 0);
        return;
    }

    /* Get length of data pending in circular buffer */
    to_send = uart_circ_chars_pending(xmit);

    if (likely(to_send)) {
        /* Limit to size of TX FIFO */
        txlen = CH43X_FIFO_SIZE;
        to_send = (to_send > txlen) ? txlen : to_send;

        /* Add data to send */
        port->icount.tx += to_send;

        /* Convert to linear buffer */
        for (i = 0; i < to_send; ++i) {
            s->buf[i] = xmit->buf[xmit->tail];
            xmit->tail = (xmit->tail + 1) & (UART_XMIT_SIZE - 1);
        }
        dev_dbg(&s->spi_dev->dev, "ch43x_handle_tx %d bytes\n", to_send);
        thr_reg = 0x02 | ((CH43X_THR_REG + port->line * 0x08) << CH43X_REG_SHIFT);
        ch43x_raw_write(port, &thr_reg, s->buf, to_send);
    }

    if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
        uart_write_wakeup(port);
}

static void ch43x_port_irq(struct ch43x_port *s, int portno)
{
    struct uart_port *port = &s->p[portno].port;

    do {
        unsigned int iir, msr;
        unsigned char lsr;
        lsr = ch43x_port_read(port, CH43X_LSR_REG);
        if (lsr & 0x02) {
            port->icount.overrun++;
            dev_err(port->dev, "Rx Overrun portno = %d, lsr = 0x%2x\n", portno, lsr);
        }

        iir = ch43x_port_read(port, CH43X_IIR_REG);
        if (iir & CH43X_IIR_NO_INT_BIT) {
            dev_dbg(&s->spi_dev->dev, "%s no int, quit\n", __func__);
            break;
        }
        iir &= CH43X_IIR_ID_MASK;
        switch (iir) {
        case CH43X_IIR_RDI_SRC:
        case CH43X_IIR_RLSE_SRC:
        case CH43X_IIR_RTOI_SRC:
            ch43x_handle_rx(port, iir);
            break;
        case CH43X_IIR_MSI_SRC:
            msr = ch43x_port_read(port, CH43X_MSR_REG);
            s->p[portno].msr_reg = msr;
            dev_dbg(&s->spi_dev->dev, "uart_handle_modem_change = 0x%02x\n", msr);
            break;
        case CH43X_IIR_THRI_SRC:
            mutex_lock(&s->mutex);
            ch43x_handle_tx(port);
            mutex_unlock(&s->mutex);
            break;
        default:
            dev_err(port->dev, "Port %i: Unexpected interrupt: %x", port->line, iir);
            break;
        }
    } while (1);
}

static irqreturn_t ch43x_ist_top(int irq, void *dev_id)
{
    return IRQ_WAKE_THREAD;
}

static irqreturn_t ch43x_ist(int irq, void *dev_id)
{
    struct ch43x_port *s = (struct ch43x_port *)dev_id;
    int i;

    dev_dbg(&s->spi_dev->dev, "ch43x_ist interrupt enter...\n");

    for (i = 0; i < s->uart.nr; ++i)
        ch43x_port_irq(s, i);

    dev_dbg(&s->spi_dev->dev, "%s end\n", __func__);

    return IRQ_HANDLED;
}

static void ch43x_wq_proc(struct work_struct *ws)
{
    struct ch43x_one *one = to_ch43x_one(ws, tx_work);
    struct ch43x_port *s = dev_get_drvdata(one->port.dev);

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
    mutex_lock(&s->mutex);
    ch43x_port_update(&one->port, CH43X_IER_REG, CH43X_IER_THRI_BIT, CH43X_IER_THRI_BIT);
    mutex_unlock(&s->mutex);
}

static void ch43x_stop_tx(struct uart_port *port)
{
    struct ch43x_one *one = to_ch43x_one(port, port);
    struct ch43x_port *s = dev_get_drvdata(one->port.dev);

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
    schedule_work(&one->stop_tx_work);
}

static void ch43x_stop_rx(struct uart_port *port)
{
    struct ch43x_one *one = to_ch43x_one(port, port);
    struct ch43x_port *s = dev_get_drvdata(one->port.dev);

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
    schedule_work(&one->stop_rx_work);
}

static void ch43x_start_tx(struct uart_port *port)
{
    struct ch43x_one *one = to_ch43x_one(port, port);
    struct ch43x_port *s = dev_get_drvdata(one->port.dev);

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

static void ch43x_stop_rx_work_proc(struct work_struct *ws)
{
    struct ch43x_one *one = to_ch43x_one(ws, stop_rx_work);
    struct ch43x_port *s = dev_get_drvdata(one->port.dev);

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
    mutex_lock(&s->mutex);
    one->port.read_status_mask &= ~CH43X_LSR_DR_BIT;
    ch43x_port_update(&one->port, CH43X_IER_REG, CH43X_IER_RDI_BIT, 0);
    ch43x_port_update(&one->port, CH43X_IER_REG, CH43X_IER_RLSI_BIT, 0);
    mutex_unlock(&s->mutex);
}

static void ch43x_stop_tx_work_proc(struct work_struct *ws)
{
    struct ch43x_one *one = to_ch43x_one(ws, stop_tx_work);
    struct ch43x_port *s = dev_get_drvdata(one->port.dev);
    struct circ_buf *xmit = &one->port.state->xmit;

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
    mutex_lock(&s->mutex);
    /* handle rs485 */
    if (one->rs485.flags & SER_RS485_ENABLED) {
        /* do nothing if current tx not yet completed */
        int lsr = ch43x_port_read(&one->port, CH43X_LSR_REG);
        if (!(lsr & CH43X_LSR_TEMT_BIT)) {
            mutex_unlock(&s->mutex);
            return;
        }
        if (uart_circ_empty(xmit) && (one->rs485.delay_rts_after_send > 0))
            mdelay(one->rs485.delay_rts_after_send);
    }

    ch43x_port_update(&one->port, CH43X_IER_REG, CH43X_IER_THRI_BIT, 0);
    mutex_unlock(&s->mutex);
}

static unsigned int ch43x_tx_empty(struct uart_port *port)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    unsigned int lsr;
    unsigned int result;

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
    lsr = ch43x_port_read(port, CH43X_LSR_REG);
    result = (lsr & CH43X_LSR_THRE_BIT) ? TIOCSER_TEMT : 0;

    return result;
}

static unsigned int ch43x_get_mctrl(struct uart_port *port)
{
    unsigned int status, ret;
    struct ch43x_port *s = dev_get_drvdata(port->dev);

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

static void ch43x_md_proc(struct work_struct *ws)
{
    struct ch43x_one *one = to_ch43x_one(ws, md_work);
    struct ch43x_port *s = dev_get_drvdata(one->port.dev);
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

    ch43x_port_write(&one->port, CH43X_MCR_REG, mcr);
}

static void ch43x_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
    struct ch43x_one *one = to_ch43x_one(port, port);
    struct ch43x_port *s = dev_get_drvdata(one->port.dev);

    dev_dbg(&s->spi_dev->dev, "%s - mctrl:0x%x\n", __func__, mctrl);
    schedule_work(&one->md_work);
}

static void ch43x_break_ctl(struct uart_port *port, int break_state)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
    ch43x_port_update(port, CH43X_LCR_REG, CH43X_LCR_TXBREAK_BIT, break_state ? CH43X_LCR_TXBREAK_BIT : 0);
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0))
static void ch43x_set_termios(struct uart_port *port, struct ktermios *termios, const struct ktermios *old)
#else
static void ch43x_set_termios(struct uart_port *port, struct ktermios *termios, struct ktermios *old)
#endif
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    struct ch43x_one *one = to_ch43x_one(port, port);
    unsigned int lcr;
    int baud;
    u8 bParityType;

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);

    /* Mask termios capabilities we don't support */
    termios->c_cflag &= ~CMSPAR;

    /* Word size */
    switch (termios->c_cflag & CSIZE) {
    case CS5:
        lcr = CH43X_LCR_WORD_LEN_5;
        break;
    case CS6:
        lcr = CH43X_LCR_WORD_LEN_6;
        break;
    case CS7:
        lcr = CH43X_LCR_WORD_LEN_7;
        break;
    case CS8:
        lcr = CH43X_LCR_WORD_LEN_8;
        break;
    default:
        lcr = CH43X_LCR_WORD_LEN_8;
        termios->c_cflag &= ~CSIZE;
        termios->c_cflag |= CS8;
        break;
    }

    bParityType =
        termios->c_cflag & PARENB ? (termios->c_cflag & PARODD ? 1 : 2) + (termios->c_cflag & CMSPAR ? 2 : 0) : 0;
    lcr |= CH43X_LCR_PARITY_BIT;
    switch (bParityType) {
    case 0x01:
        lcr |= CH43X_LCR_ODDPARITY_BIT;
        dev_dbg(&s->spi_dev->dev, "parity = odd\n");
        break;
    case 0x02:
        lcr |= CH43X_LCR_EVENPARITY_BIT;
        dev_dbg(&s->spi_dev->dev, "parity = even\n");
        break;
    case 0x03:
        lcr |= CH43X_LCR_MARKPARITY_BIT;
        dev_dbg(&s->spi_dev->dev, "parity = mark\n");
        break;
    case 0x04:
        lcr |= CH43X_LCR_SPACEPARITY_BIT;
        dev_dbg(&s->spi_dev->dev, "parity = space\n");
        break;
    default:
        lcr &= ~CH43X_LCR_PARITY_BIT;
        dev_dbg(&s->spi_dev->dev, "parity = none\n");
        break;
    }

    /* Stop bits */
    if (termios->c_cflag & CSTOPB)
        lcr |= CH43X_LCR_STOPLEN_BIT; /* 2 stops */

    /* Set read status mask */
    port->read_status_mask = CH43X_LSR_OE_BIT;
    if (termios->c_iflag & INPCK)
        port->read_status_mask |= CH43X_LSR_PE_BIT | CH43X_LSR_FE_BIT;
    if (termios->c_iflag & (BRKINT | PARMRK))
        port->read_status_mask |= CH43X_LSR_BI_BIT;

    /* Set status ignore mask */
    port->ignore_status_mask = 0;
    if (termios->c_iflag & IGNBRK)
        port->ignore_status_mask |= CH43X_LSR_BI_BIT;
    if (!(termios->c_cflag & CREAD))
        port->ignore_status_mask |= CH43X_LSR_BRK_ERROR_MASK;

    /* Update LCR register */
    ch43x_port_write(port, CH43X_LCR_REG, lcr);

    /* Configure flow control */
    if (termios->c_cflag & CRTSCTS) {
        dev_dbg(&s->spi_dev->dev, "ch43x_set_termios enable rts/cts\n");
        ch43x_port_update(port, CH43X_MCR_REG, CH43X_MCR_AFE | CH43X_MCR_RTS_BIT, CH43X_MCR_AFE | CH43X_MCR_RTS_BIT);
        one->mcr_force |= CH43X_MCR_AFE | CH43X_MCR_RTS_BIT;
        // add on 20200608 suppose cts status is always valid here
        uart_handle_cts_change(port, 1);
    } else {
        dev_dbg(&s->spi_dev->dev, "ch43x_set_termios disable rts/cts\n");
        ch43x_port_update(port, CH43X_MCR_REG, CH43X_MCR_AFE, 0);
        one->mcr_force &= ~(CH43X_MCR_AFE | CH43X_MCR_RTS_BIT);
    }

    /* Get baud rate generator configuration */
    baud = uart_get_baud_rate(port, termios, old, port->uartclk / 16 / 0xffff, port->uartclk / 16 * 24);
    /* Setup baudrate generator */
    baud = ch43x_set_baud(port, baud);
    /* Update timeout according to new baud rate */
    uart_update_timeout(port, termios->c_cflag, baud);
}

static int ch43x_startup(struct uart_port *port)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    unsigned int val;
    struct ch43x_one *one = to_ch43x_one(port, port);

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);

    /* Reset FIFOs*/
    val = CH43X_FCR_RXRESET_BIT | CH43X_FCR_TXRESET_BIT;
    ch43x_port_write(port, CH43X_FCR_REG, val);
    udelay(5);
    /* Enable FIFOs and configure interrupt & flow control levels to 8 */
    ch43x_port_write(port, CH43X_FCR_REG, CH43X_FCR_RXLVLH_BIT | CH43X_FCR_FIFO_BIT);

    /* Now, initialize the UART */
    ch43x_port_write(port, CH43X_LCR_REG, CH43X_LCR_WORD_LEN_8);

    /* Enable RX, CTS change interrupts */
    val = ch43x_port_read(port, CH43X_IER_REG);
    val |= CH43X_IER_RDI_BIT | CH43X_IER_RLSI_BIT | CH43X_IER_MSI_BIT;
    ch43x_port_write(port, CH43X_IER_REG, val);

    /* Enable Uart interrupts */
    ch43x_port_write(port, CH43X_MCR_REG, CH43X_MCR_OUT2);
    one->mcr_force = CH43X_MCR_OUT2;

    return 0;
}

static void ch43x_shutdown(struct uart_port *port)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    struct ch43x_one *one = to_ch43x_one(port, port);

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);

    /* Disable uart0 interrupts */
    if (port->line == 0)
        ch43x_port_write(port, CH43X_IER_REG, 0);
    ch43x_port_write(port, CH43X_MCR_REG, 0);

    one->mcr_force = 0;
}

static const char *ch43x_type(struct uart_port *port)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);
    return (port->type == PORT_SC16IS7XX) ? s->devtype->name : NULL;
}

static int ch43x_request_port(struct uart_port *port)
{
    /* Do nothing */
    return 0;
}

static void ch43x_config_port(struct uart_port *port, int flags)
{
    if (flags & UART_CONFIG_TYPE)
        port->type = PORT_SC16IS7XX;
}

static int ch43x_verify_port(struct uart_port *port, struct serial_struct *s)
{
    if ((s->type != PORT_UNKNOWN) && (s->type != PORT_SC16IS7XX))
        return -EINVAL;
    if (s->irq != port->irq)
        return -EINVAL;

    return 0;
}

static void ch43x_pm(struct uart_port *port, unsigned int state, unsigned int oldstate)
{
    struct ch43x_port *s = dev_get_drvdata(port->dev);

    dev_dbg(&s->spi_dev->dev, "%s\n", __func__);
}

static void ch43x_null_void(struct uart_port *port)
{
    /* Do nothing */
}
static void ch43x_enable_ms(struct uart_port *port)
{
    /* Do nothing */
}

static const struct uart_ops ch43x_ops = {
    .tx_empty = ch43x_tx_empty,
    .set_mctrl = ch43x_set_mctrl,
    .get_mctrl = ch43x_get_mctrl,
    .stop_tx = ch43x_stop_tx,
    .start_tx = ch43x_start_tx,
    .stop_rx = ch43x_stop_rx,
    .break_ctl = ch43x_break_ctl,
    .startup = ch43x_startup,
    .shutdown = ch43x_shutdown,
    .set_termios = ch43x_set_termios,
    .type = ch43x_type,
    .request_port = ch43x_request_port,
    .release_port = ch43x_null_void,
    .config_port = ch43x_config_port,
    .verify_port = ch43x_verify_port,
    .enable_ms = ch43x_enable_ms,
    .pm = ch43x_pm,
};

static ssize_t ch43x_proc_read(struct file *file, char __user *user_buf, size_t count, loff_t *ppos)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(6, 0, 0))
    struct ch43x_port *s = PDE_DATA(file_inode(file));
#else
    struct ch43x_port *s = pde_data(file_inode(file));
#endif
    char *buf;
    int i, j, len, ret;
    u8 lcr;

    buf = kzalloc(4096, GFP_KERNEL);
    if (!buf)
        return 0;

    len += snprintf(buf + len, REGS_BUFSIZE - len, "============ch432 registers:============\n");

    for (i = 0; i < s->devtype->nr_uart; i++) {
        lcr = ch43x_port_read(&s->p[i].port, CH43X_LCR_REG);
        ch43x_port_write(&s->p[i].port, CH43X_LCR_REG, lcr | CH43X_LCR_DLAB_BIT);
        len += snprintf(buf + len, REGS_BUFSIZE - len, "============UART%d Dump register at DLAB=1============\n", i);
        for (j = 0; j < 2; j++) {
            len += snprintf(buf + len, REGS_BUFSIZE - len, "reg:0x%02x val:0x%02x\n", j,
                            ch43x_port_read(&s->p[i].port, j));
        }
        ch43x_port_write(&s->p[i].port, CH43X_LCR_REG, lcr);

        len += snprintf(buf + len, REGS_BUFSIZE - len, "============UART%d Dump register at DLAB=0============\n", i);
        for (j = 0; j < 8; j++) {
            len += snprintf(buf + len, REGS_BUFSIZE - len, "reg:0x%02x val:0x%02x\n", j,
                            ch43x_port_read(&s->p[i].port, j));
        }
    }

    ret = simple_read_from_buffer(user_buf, count, ppos, buf, len);
    kfree(buf);

    return ret;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 6, 0))
static const struct file_operations ch43x_regs_ops = {
    .owner = THIS_MODULE,
    .read = ch43x_proc_read,
};
#else
static const struct proc_ops ch43x_regs_ops = {
    .proc_read = ch43x_proc_read,
};
#endif

int ch43x_debugfs_init(struct ch43x_port *s)
{
    struct proc_dir_entry *proc_entry;

    snprintf(s->proc_file_name, sizeof(s->proc_file_name), "ch43x%d_reg", s->minor);
    proc_entry = proc_create_data(s->proc_file_name, S_IRUGO | S_IWUGO, NULL, &ch43x_regs_ops, s);
    if (!proc_entry) {
        dev_err(&s->spi_dev->dev, "Failed to create /proc/%s\n", s->proc_file_name);
        return -ENOMEM;
    }

    return 0;
}

void ch43x_debugfs_exit(struct ch43x_port *s)
{
    remove_proc_entry(s->proc_file_name, NULL);
}

static int ch43x_probe(struct spi_device *spi, struct ch43x_devtype *devtype, int irq, unsigned long flags)
{
    unsigned long freq;
    int i, ret;
    struct ch43x_port *s;
    struct device *dev = &spi->dev;
    const char *ch43x_uart_name[] = {"ttyCH43X",  "ttyCH43XA", "ttyCH43XB", "ttyCH43XC", "ttyCH43XD",
                                     "ttyCH43XE", "ttyCH43XF", "ttyCH43XG", "ttyCH43XH"};

    /* Alloc port structure */
    s = devm_kzalloc(dev, sizeof(*s) + sizeof(struct ch43x_one) * devtype->nr_uart, GFP_KERNEL);
    if (!s) {
        dev_err(dev, "Error allocating port structure\n");
        return -ENOMEM;
    }

    s->minor = ch43x_alloc_minor(s);
    if (s->minor >= CH43X_MAX_CNT)
        return -ENOMEM;

    /* 22.1184Mhz Crystal by default, uart clock is processed to double frequency, refer CH432DS1.PDF chapter 5.2 */
    freq = CRYSTAL_FREQ * 2;
    s->devtype = devtype;
    dev_set_drvdata(dev, s);
    s->spi_dev = spi;

    ret = ch43x_spi_test(s);
    if (ret < 0)
        return -1;

    /* Register UART driver */
    s->uart.owner = THIS_MODULE;
    s->uart.dev_name = ch43x_uart_name[s->minor];
    s->uart.nr = devtype->nr_uart;
    ret = uart_register_driver(&s->uart);
    if (ret) {
        dev_err(dev, "Registering UART driver failed\n");
        goto out_clk;
    }

    mutex_init(&s->mutex);
    mutex_init(&s->mutex_bus_access);
    for (i = 0; i < devtype->nr_uart; ++i) {
        /* Initialize port data */
        s->p[i].port.line = i;
        s->p[i].port.dev = dev;
        s->p[i].port.irq = irq;
        s->p[i].port.type = PORT_SC16IS7XX;
        s->p[i].port.fifosize = CH43X_FIFO_SIZE;
        s->p[i].port.flags = UPF_FIXED_TYPE | UPF_LOW_LATENCY;
        s->p[i].port.iotype = UPIO_PORT;
        s->p[i].port.uartclk = freq;
        s->p[i].port.ops = &ch43x_ops;
        /* Disable all interrupts */
        ch43x_port_write(&s->p[i].port, CH43X_IER_REG, 0);
        /* Disable uart interrupts */
        ch43x_port_write(&s->p[i].port, CH43X_MCR_REG, 0);

        s->p[i].msr_reg = ch43x_port_read(&s->p[i].port, CH43X_MSR_REG);

        /* Initialize queue for start TX */
        INIT_WORK(&s->p[i].tx_work, ch43x_wq_proc);
        /* Initialize queue for changing mode */
        INIT_WORK(&s->p[i].md_work, ch43x_md_proc);

        INIT_WORK(&s->p[i].stop_rx_work, ch43x_stop_rx_work_proc);
        INIT_WORK(&s->p[i].stop_tx_work, ch43x_stop_tx_work_proc);

        /* Register port */
        ret = uart_add_one_port(&s->uart, &s->p[i].port);
        if (ret < 0) {
            dev_err(&s->spi_dev->dev, "Failed to add UART: %d\n", ret);
            goto out;
        }
        dev_info(&s->spi_dev->dev, "ttyCH43X%d: uart device\n", i);
    }

    ch43x_port_update_specify(&s->p[1].port, 1, CH43X_IER_REG, CH43X_IER_CK2X_BIT, CH43X_IER_CK2X_BIT);
    ret = devm_request_threaded_irq(dev, irq, ch43x_ist_top, ch43x_ist, IRQF_ONESHOT | flags, dev_name(dev), s);
    if (ret < 0) {
        dev_err(dev, "irq %d request failed, error %d\n", irq, ret);
        goto out;
    }

    ch43x_debugfs_init(s);

    return 0;
out:
    mutex_destroy(&s->mutex);
    uart_unregister_driver(&s->uart);
out_clk:
    if (!IS_ERR(s->clk))
        /*clk_disable_unprepare(s->clk)*/;

    return ret;
}

static int ch43x_remove(struct device *dev)
{
    struct ch43x_port *s = dev_get_drvdata(dev);
    int i;

    dev_dbg(dev, "%s\n", __func__);

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

    ch43x_debugfs_exit(s);
    ch43x_release_minor(s);

    return 0;
}

const struct of_device_id __maybe_unused ch43x_dt_ids[] = {
    {
        .compatible = "wch,ch43x",
        .data = &ch43x_devtype,
    },
    { },
};
MODULE_DEVICE_TABLE(of, ch43x_dt_ids);

int ch43x_spi_probe(struct spi_device *spi)
{
    struct ch43x_devtype *devtype = &ch43x_devtype;
    unsigned long flags = IRQF_TRIGGER_LOW;
    int ret;
    u32 save;

    save = spi->mode;
    spi->mode |= SPI_MODE_3;
    if (spi_setup(spi) < 0) {
        spi->mode = save;
    } else {
        dev_dbg(&spi->dev, "change to SPI MODE 3!\n");
    }

#ifdef USE_IRQ_FROM_DTS
    /* if your platform supports acquire irq number from dts */
    ret = ch43x_probe(spi, devtype, spi->irq, flags);
#else
    ret = devm_gpio_request(dev, GPIO_NUMBER, "gpioint");
    if (ret) {
        dev_err(dev, "Failed request gpio:%d\n", GPIO_NUMBER);
        return ret;
    }
    ret = gpio_direction_input(GPIO_NUMBER);
    if (ret) {
        dev_err(dev, "Failed set gpio direction\n");
        return ret;
    }
    irq_set_irq_type(gpio_to_irq(GPIO_NUMBER), flags);
    ret = ch43x_probe(spi, devtype, gpio_to_irq(GPIO_NUMBER), flags);
#endif
    return ret;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
void ch43x_spi_remove(struct spi_device *spi)
#else
int ch43x_spi_remove(struct spi_device *spi)
#endif
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 18, 0))
    ch43x_remove(&spi->dev);
#else
    return ch43x_remove(&spi->dev);
#endif
}

static struct spi_driver ch43x_spi_driver = {
    .driver = {
        .name = "ch43x",
        .bus = &spi_bus_type,
        .owner = THIS_MODULE,
        .of_match_table = of_match_ptr(ch43x_dt_ids),
    },
    .probe = ch43x_spi_probe,
    .remove = ch43x_spi_remove,
};

static int __init ch43x_init(void)
{
    printk(KERN_INFO KBUILD_MODNAME ": " DRIVER_DESC "\n");
    printk(KERN_INFO KBUILD_MODNAME ": " VERSION_DESC "\n");

    return spi_register_driver(&ch43x_spi_driver);
}

static void __exit ch43x_exit(void)
{
    spi_unregister_driver(&ch43x_spi_driver);
}

module_init(ch43x_init);
module_exit(ch43x_exit);

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
